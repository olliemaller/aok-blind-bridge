# ESPHome component: `aok_rf`

If you already run ESPHome, you can use the radio as an ESPHome component instead of flashing the bridge firmware. You get ESPHome's native Home Assistant API, OTA and YAML. You don't get the bridge's web dashboard, learning page or calibration helper; everything else can be done in YAML (see the table at the end).

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

## `on_long_press`

Fires when a button on one of your remotes is let go after being held for 1.5 seconds or more. Variables: `action` (`up`, `down` or `stop`), `channels` (mask) and `seconds` (`float`, how long it was held).

```yaml
aok_rf:
  # …
  on_long_press:
    - if:
        condition:
          lambda: 'return action == "up" && channels == 0x0001;'
        then:
          - script.execute: scene_morning
```

## More remotes

`extra_remotes:` takes a list of `prefix` and `remote_id`. Their presses set `my_remote` to true in `on_frame` and count for `on_long_press`, so a second hand remote can drive the same covers. The bridge only sends with the main remote's codes.

## Signal strength

```yaml
aok_rf:
  rssi:
    name: Remote signal
```

A sensor in dBm, updated with each press heard from your remotes. Above about -70 is good; below -90 the bridge will miss presses.

## Repeater

`repeat_remote: true` sends each hand-remote press again once the button is let go. Use it when a motor is at the edge of the hand remote's range and the ESP32 sits between them.

## Frequency scan

```yaml
button:
  - platform: template
    name: Frequency scan (hold a remote button)
    entity_category: diagnostic
    on_press:
      - aok_rf.frequency_scan: rf
```

Hold any button on the hand remote near the ESP32 and press the button in Home Assistant. It listens from 433.42 to 434.42 MHz for about 4 seconds and logs the strongest frequency:

```
[I][aok_rf]: Frequency scan: strongest at 433.87 MHz (-41 dBm); set frequency: 433.87MHz to use it
```

Put that in `frequency:` and reflash. Home Assistant shows the device as unavailable for the 4 seconds while it scans.

## Pairing a motor to the component's own ID (experimental)

This gives the ESP32 its own remote ID for a channel, so that motor keeps working from Home Assistant if the hand remote is lost or replaced. The hand remote keeps working too. It hasn't been tested on every A-OK motor; if the motor doesn't jog at the end, nothing has changed.

1. Press **P** on the back of the hand remote once (with the channel selected). The log shows `P button heard -> remote: program: 0xAC40`. Add that `program:` line to `remote:`.
2. Pick an unused ID and add `own_id: 0x99` to `aok_rf:`. Flash.
3. Run `aok_rf.pair: { id: rf, channels: 3 }` (a template button is easiest). The ESP32 sends P twice from the hand remote's ID (the motor jogs each time), then Up from its own ID (the motor jogs to confirm).
4. If it jogged, add the channel to `own_channels: [3]` and flash. From then on `aok_rf.send` uses the own ID for that channel; other channels in the same `channels:` list still go out with the hand remote's ID.

No `program:` code? Press P twice on the hand remote yourself, then run `aok_rf.pair: { id: rf, channels: 3, pressed_p: true }` within about 10 seconds.

## Bridge firmware features in ESPHome

[`esphome/full-example.yaml`](../esphome/full-example.yaml) shows every one of these working together. CI checks it and compiles it on every change, along with the smaller [`esphome/example.yaml`](../esphome/example.yaml).

| Bridge firmware | ESPHome |
|---|---|
| Blinds with travel times, position | `time_based` cover |
| Groups (one frame for several channels) | `channels: [1, 2, 3]`, or a script over several covers |
| Named positions / favourite | template button with `cover.control` and `position:` |
| Scenes | `script:` with several `cover.control` steps |
| Schedules, sunrise/sunset | `time:` `on_time`, `sun:` `on_sunrise` / `on_sunset` |
| Lock | template switch, checked in `open_action` / `close_action` |
| Window interlock | HA `binary_sensor`, checked in `close_action` |
| Wind protection | HA `binary_sensor` with `on_press: cover.close` |
| Follow the hand remote | `on_frame` |
| Long press | `on_long_press` |
| Extra remotes | `extra_remotes:` |
| Signal strength | `rssi:` |
| Repeater | `repeat_remote: true` |
| Frequency scan | `aok_rf.frequency_scan` |
| Pair to own ID | `own_id:`, `own_channels:`, `aok_rf.pair` |
| Tilt (venetian slats) | Not with `time_based`. ESPHome's [`feedback`](https://esphome.io/components/cover/feedback/) or a template cover can do it with a lambda; the bridge firmware is simpler. |
| Wired Ethernet | ESPHome's own [`ethernet:`](https://esphome.io/components/ethernet/) block (WT32-ETH01: `type: LAN8720`, `mdc_pin: GPIO23`, `mdio_pin: GPIO18`, `clk_mode: GPIO0_IN`, `phy_addr: 1`, `power_pin: GPIO16`) with the CC1101 on the pins in [setup](setup.md#ethernet-wt32-eth01) |
| Web dashboard, learning page, calibration helper | Not available; use the logs and HA |
