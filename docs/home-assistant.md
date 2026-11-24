# Home Assistant

The bridge uses MQTT discovery. MQTT is a simple messaging system: a broker (a small server, usually the Mosquitto add-on on Home Assistant) passes messages between the bridge and Home Assistant, and discovery messages tell Home Assistant which entities exist. Once the bridge is connected to your broker, the blinds appear under **Settings → Devices & services → MQTT**, with no YAML. Each blind is its own device, so it can sit in its own area. If you set a room for the blind, Home Assistant suggests that area.

## Entities

**Each blind**

| Entity | What it does |
|---|---|
| `cover.<blind>` | Open, close, stop, and set position (0 to 100 %). With a *Tilt time* set, also a tilt slider |
| *Favourite position* button | Moves to the favourite % (only if one is set) |
| *Mark as open* / *Mark as closed* buttons | Set the position without moving the blind |
| Named position buttons | One per named position (up to 4 per blind, set in *Set up blinds*) |
| *Lock* switch | While on, Home Assistant, scenes and schedules leave the blind alone. The hand remote and Stop still work |
| Scenes | One `scene.` entity per scene from *Scenes & schedules* |

The cover's attributes include `channels`, `last_source` (`web`, `home_assistant`, `hand_remote` and so on), the travel times and `position_known`.

**The bridge**

| Entity | What it does |
|---|---|
| `event.<bridge>_remote_button` | Fires `open`, `close` or `stop` with a `channels` attribute whenever your hand remote is used, and `open_long`, `close_long` or `stop_long` (with `seconds`) when a button was held for 1.5 s or more. A followed Somfy remote fires `open`, `close` or `stop` with `somfy_remote` (its address) and `somfy_command` (`up`, `down`, `my` and so on) instead of `channels` |
| *Remote signal* | Signal strength (dBm) of the last frame from your remotes: below about −90 the bridge is too far away |
| *Learn remote* button | Starts remote learning |
| *Last remote press* | The last button and channel heard |
| *CC1101 radio* | Connectivity: whether the radio is working |
| *Wi-Fi signal*, *Uptime*, *IP address* | Diagnostics |

## Using the remote for automations

The remote-button event can also run automations. For example, turn the lounge lights on whenever channel 1's Up is pressed:

```yaml
triggers:
  - trigger: state
    entity_id: event.a_ok_blind_bridge_remote_button
    not_from: unavailable
conditions:
  - condition: template
    value_template: >
      {{ trigger.to_state.attributes.event_type == 'open'
         and trigger.to_state.attributes.channels == '1' }}
actions:
  - action: light.turn_on
    target:
      area_id: lounge
```

## MQTT topics

`<base>` is `aok_blinds` by default. You can change it under *Settings → Home Assistant → Topics*.

| Topic | Direction | Payload |
|---|---|---|
| `<base>/<blind>/set` | to bridge | `OPEN`, `CLOSE`, `STOP`, `FAVOURITE`, `SYNC_OPEN`, `SYNC_CLOSE` or `0` to `100` |
| `<base>/<blind>/set_position` | to bridge | `0` to `100` |
| `<base>/all/set` | to bridge | `OPEN`, `CLOSE` or `STOP` for every blind at once |
| `<base>/learn/set` | to bridge | `START` |
| `<base>/<blind>/set` | to bridge | also `PRESET:1` to `PRESET:4` or `PRESET:<name>`, `LOCK`, `UNLOCK` |
| `<base>/<blind>/lock/set` | to bridge | `ON` or `OFF` |
| `<base>/<blind>/tilt/set` | to bridge | `0` (closed) to `100` (open). Only for blinds with a *Tilt time*. `TILT:<0-100>` on `<base>/<blind>/set` does the same |
| `<base>/<blind>/interlock` | to bridge | `ON` while the window or door is open (the blind won't close, and stops if closing), `OFF` when shut. Send it retained |
| `<base>/wind` | to bridge | `ON`: every *Awning* retracts and stays until `OFF`. Send it retained |
| `<base>/scene/set` | to bridge | a scene's id or name |
| `<base>/channels/set` | to bridge | `1,3:OPEN`, `2-5:CLOSE` or `4:STOP`: any channels in one radio frame |
| `<base>/<blind>/lock` | from bridge | `ON` or `OFF` |
| `<base>/<blind>/available` | from bridge | `online` when the radio works and the remote is learned (or the blind is a Somfy motor) |
| `<base>/<blind>/tilt` | from bridge | `0` to `100`, once the tilt is known. Only for blinds with a *Tilt time* |
| `<base>/<blind>/state` | from bridge | `open`, `closed`, `opening`, `closing` or `stopped` |
| `<base>/<blind>/position` | from bridge | `0` to `100` |
| `<base>/<blind>/attributes` | from bridge | JSON |
| `<base>/remote/event` | from bridge | `{"event_type":"open","channels":"1"}`. A long press: `{"event_type":"open_long","seconds":2.1}`. A Somfy remote: `{"event_type":"close","somfy_remote":"0F12AB","somfy_command":"down"}` |
| `<base>/diag/rssi` | from bridge | Wi-Fi signal in dBm |
| `<base>/diag/uptime` | from bridge | seconds since the bridge started |
| `<base>/diag/ip` | from bridge | the bridge's IP address |
| `<base>/diag/radio` | from bridge | `ON` when the CC1101 is working, `OFF` if not |
| `<base>/diag/last_remote` | from bridge | the last button heard, for example `open · channel 1` |
| `<base>/diag/radio_rssi` | from bridge | signal strength of the last frame from your remotes, in dBm |
| `<base>/status` | from bridge | `online` or `offline` (last will) |

## Positions

The motors don't report where they are. The bridge works out each blind's position from how long it has been moving, using its opening and closing times. Moves to the end stops (fully open or closed) run a little longer than the travel time, so the motor stops itself at its limit. Moves to a middle position send Stop when the time is up.

If a blind gets out of step (for example, it was moved while the bridge was off), press **Mark as open** or **Mark as closed**, or just open or close it fully once.

## Window and wind examples

Stop a blind closing over an open window:

```yaml
triggers:
  - trigger: state
    entity_id: binary_sensor.lounge_window
actions:
  - action: mqtt.publish
    data:
      topic: aok_blinds/lounge_left/interlock
      payload: "{{ 'ON' if trigger.to_state.state == 'on' else 'OFF' }}"
      retain: true
```

Retract the awnings when it's windy:

```yaml
triggers:
  - trigger: numeric_state
    entity_id: sensor.wind_gust
    above: 40
    id: "ON"
  - trigger: numeric_state
    entity_id: sensor.wind_gust
    below: 25
    for: "00:15:00"
    id: "OFF"
actions:
  - action: mqtt.publish
    data:
      topic: aok_blinds/wind
      payload: "{{ trigger.id }}"
      retain: true
```
