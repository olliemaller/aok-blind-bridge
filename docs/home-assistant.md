# Home Assistant

The bridge uses MQTT discovery. Once it's connected to your broker, the blinds appear under **Settings → Devices & services → MQTT**, with no YAML. Each blind is its own device, so it can sit in its own area. If you set a room for the blind, Home Assistant suggests that area.

## Entities

**Each blind**

| Entity | What it does |
|---|---|
| `cover.<blind>` | Open, close, stop, and set position (0–100 %) |
| *Favourite position* button | Moves to the favourite % (only if one is set) |
| *Mark as open* / *Mark as closed* buttons | Set the position without moving the blind |

The cover's attributes include `channels`, `last_source` (`web`, `home_assistant`, `hand_remote` and so on), the travel times and `position_known`.

**The bridge**

| Entity | What it does |
|---|---|
| `event.<bridge>_remote_button` | Fires `open`, `close` or `stop` with a `channels` attribute whenever your hand remote is used |
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
| `<base>/<blind>/set` | to bridge | `OPEN`, `CLOSE`, `STOP`, `FAVOURITE`, `SYNC_OPEN`, `SYNC_CLOSE` or `0`–`100` |
| `<base>/<blind>/set_position` | to bridge | `0`–`100` |
| `<base>/all/set` | to bridge | `OPEN`, `CLOSE` or `STOP` for every blind at once |
| `<base>/learn/set` | to bridge | `START` |
| `<base>/<blind>/state` | from bridge | `open`, `closed`, `opening`, `closing` or `stopped` |
| `<base>/<blind>/position` | from bridge | `0`–`100` |
| `<base>/<blind>/attributes` | from bridge | JSON |
| `<base>/remote/event` | from bridge | `{"event_type":"open","channels":"1"}` |
| `<base>/status` | from bridge | `online` or `offline` (last will) |

## Positions

The motors don't report where they are. The bridge works out each blind's position from how long it has been moving, using its opening and closing times. Moves to the end stops (fully open or closed) run a little longer than the travel time, so the motor stops itself at its limit. Moves to a middle position send Stop when the time is up.

If a blind gets out of step (for example, it was moved while the bridge was off), press **Mark as open** or **Mark as closed**, or just open or close it fully once.
