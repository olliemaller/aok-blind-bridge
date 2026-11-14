"""ESPHome component for 433.92 MHz A-OK blind motors through a CC1101 radio.

Sends Up/Down/Stop for any channels of a learned remote, and fires on_frame for
every A-OK frame heard (your remote, other remotes, and buttons like P).
"""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.const import CONF_ID, CONF_FREQUENCY, CONF_TRIGGER_ID

CODEOWNERS = ["@olliemaller"]

aok_ns = cg.esphome_ns.namespace("aok_rf")
AokRf = aok_ns.class_("AokRf", cg.Component)
SendAction = aok_ns.class_("SendAction", automation.Action, cg.Parented.template(AokRf))
FrameTrigger = aok_ns.class_(
    "FrameTrigger", automation.Trigger.template(cg.std_string, cg.uint16, cg.bool_)
)

CONF_SCK_PIN = "sck_pin"
CONF_MISO_PIN = "miso_pin"
CONF_MOSI_PIN = "mosi_pin"
CONF_CS_PIN = "cs_pin"
CONF_GDO0_PIN = "gdo0_pin"
CONF_GDO2_PIN = "gdo2_pin"
CONF_TX_POWER = "tx_power"
CONF_REMOTE = "remote"
CONF_PREFIX = "prefix"
CONF_REMOTE_ID = "remote_id"
CONF_UP = "up"
CONF_DOWN = "down"
CONF_STOP = "stop"
CONF_REPEATS = "repeats"
CONF_ON_FRAME = "on_frame"
CONF_CHANNELS = "channels"
CONF_ACTION = "action"

ACTIONS = {"up": 0, "down": 1, "stop": 2}

pin_number = cv.int_range(min=0, max=39)
hex16 = cv.All(cv.hex_int, cv.int_range(min=0, max=0xFFFF))

REMOTE_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_PREFIX): cv.All(cv.hex_int, cv.int_range(min=0, max=0xFFFFFF)),
        cv.Required(CONF_REMOTE_ID): cv.All(cv.hex_int, cv.int_range(min=0, max=0xFF)),
        cv.Required(CONF_UP): hex16,
        cv.Required(CONF_DOWN): hex16,
        cv.Required(CONF_STOP): hex16,
    }
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(AokRf),
        cv.Optional(CONF_SCK_PIN, default=18): pin_number,
        cv.Optional(CONF_MISO_PIN, default=19): pin_number,
        cv.Optional(CONF_MOSI_PIN, default=23): pin_number,
        cv.Optional(CONF_CS_PIN, default=5): pin_number,
        cv.Optional(CONF_GDO0_PIN, default=12): pin_number,
        cv.Optional(CONF_GDO2_PIN, default=27): pin_number,
        cv.Optional(CONF_FREQUENCY, default="433.92MHz"): cv.All(
            cv.frequency, cv.Range(min=300e6, max=928e6)
        ),
        cv.Optional(CONF_TX_POWER, default=10): cv.int_range(min=-30, max=12),
        cv.Optional(CONF_REPEATS, default=8): cv.int_range(min=1, max=20),
        cv.Optional(CONF_REMOTE): REMOTE_SCHEMA,
        cv.Optional(CONF_ON_FRAME): automation.validate_automation(
            {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(FrameTrigger)}
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(
        var.set_pins(
            config[CONF_SCK_PIN],
            config[CONF_MISO_PIN],
            config[CONF_MOSI_PIN],
            config[CONF_CS_PIN],
            config[CONF_GDO0_PIN],
            config[CONF_GDO2_PIN],
        )
    )
    cg.add(var.set_frequency(config[CONF_FREQUENCY] / 1e6))
    cg.add(var.set_tx_power(config[CONF_TX_POWER]))
    cg.add(var.set_repeats(config[CONF_REPEATS]))
    if CONF_REMOTE in config:
        r = config[CONF_REMOTE]
        cg.add(var.set_remote(r[CONF_PREFIX], r[CONF_REMOTE_ID], r[CONF_UP], r[CONF_DOWN], r[CONF_STOP]))
    for conf in config.get(CONF_ON_FRAME, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(
            trigger,
            [(cg.std_string, "action"), (cg.uint16, "channels"), (cg.bool_, "my_remote")],
            conf,
        )
    cg.add_library("SPI", None)
    cg.add_library(
        "SmartRC-CC1101-Driver-Lib",
        None,
        "https://github.com/LSatan/SmartRC-CC1101-Driver-Lib.git#v3.0.2",
    )


def channel_mask(value):
    channels = cv.ensure_list(cv.int_range(min=1, max=16))(value)
    mask = 0
    for c in channels:
        mask |= 1 << (c - 1)
    return mask


@automation.register_action(
    "aok_rf.send",
    SendAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(AokRf),
            cv.Required(CONF_CHANNELS): channel_mask,
            cv.Required(CONF_ACTION): cv.enum(ACTIONS, lower=True),
        }
    ),
)
async def send_action_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_mask(config[CONF_CHANNELS]))
    cg.add(var.set_action(config[CONF_ACTION]))
    return var
