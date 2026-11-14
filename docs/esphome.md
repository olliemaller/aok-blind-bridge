# ESPHome component: `aok_rf`

If you already run ESPHome, you can use the radio as an ESPHome component instead of flashing the bridge firmware. You get ESPHome's native Home Assistant API, OTA and YAML. You don't get the bridge's dashboard, remote learning page, calibration helper, or follow-the-remote feature (you can add that one yourself; see below).

## Add it

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/olliemaller/aok-blind-bridge
      ref: main
      path: esphome/components
    components: [aok_rf]

esp32:
  board: esp32dev
  framework:
    type: arduino

aok_rf:
  id: rf
  # optional, these are the defaults (HW-395 / ESP32 DevKit wiring):
  # sck_pin: 18
  # miso_pin: 19
  # mosi_pin: 23
  # cs_pin: 5
  # gdo0_pin: 12
  # gdo2_pin: 27
  # frequency: 433.92MHz
  # tx_power: 10      # dBm, -30 to 12
  # repeats: 8        # frames per press
  remote:
    prefix: 0xA1B2C3
    remote_id: 0x3F
    up: 0xF42A
    down: 0xBCF2
    stop: 0xDC12
```

## Get your remote codes

Flash it once without the `remote:` block, open the logs, and press **Up**, **Down** and **Stop** on your remote. Each press logs a line like this:

```
[I][aok_rf]: Remote heard: up on channel 1 -> remote: prefix: 0xA1B2C3, remote_id: 0x3F, up: 0xF42A
```

Copy the prefix, the remote ID and the three codes into `remote:`. It doesn't matter which channel you press on. If you've used the bridge firmware, the same values are in the `remote` section of its backup file.

## Covers

Use ESPHome's [time-based cover](https://esphome.io/components/cover/time_based/) and send the commands with `aok_rf.send`:

```yaml
cover:
  - platform: time_based
    id: lounge_left
    name: Lounge left
    device_class: shade
    has_built_in_endstop: true
    open_duration: 24s
    close_duration: 22s
    open_action:
      - aok_rf.send: { id: rf, channels: 1, action: up }
    close_action:
      - aok_rf.send: { id: rf, channels: 1, action: down }
    stop_action:
      - aok_rf.send: { id: rf, channels: 1, action: stop }
```

`channels` takes one channel or a list (`[1, 2, 3]`). A list sends one frame that moves all of them.

## `on_frame`

Fires for every A-OK frame the radio hears. It has these variables:

| Variable | Type | Value |
|---|---|---|
| `action` | `std::string` | `up`, `down`, `stop`, `program`, `release` or `other` |
| `channels` | `uint16_t` | channel mask: bit 0 = channel 1 |
| `my_remote` | `bool` | true if it's the remote in `remote:` |

To keep a cover in step when someone uses the hand remote, send the same command to it. The cover then repeats the command over the radio, which is harmless.

```yaml
aok_rf:
  # …
  on_frame:
    - if:
        condition:
          lambda: 'return my_remote && channels == 0x0001;'   # channel 1
        then:
          - lambda: |-
              auto call = id(lounge_left).make_call();
              if (action == "up") call.set_command_open();
              else if (action == "down") call.set_command_close();
              else if (action == "stop") call.set_command_stop();
              else return;
              call.perform();
```

A full example is in [`esphome/example.yaml`](../esphome/example.yaml). CI compiles it on every change.
