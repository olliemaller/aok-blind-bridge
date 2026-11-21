# Changelog

## [1.7.0] - 2026-11-21

### Added
- **Somfy RTS motors on the same bridge.** A blind can now be a *Somfy RTS* motor (433.42 MHz) instead of an A-OK one, so a house with both kinds needs one ESP32 and one CC1101. Each Somfy blind gets its own bridge address and rolling code; pair it by holding PROG on its Somfy remote until it jogs, then pressing *Pair*. Positions, calibration, scenes, schedules, locks, the window interlock, wind protection, *All blinds* and Home Assistant discovery work the same as for A-OK blinds. Stop is sent as *My* only while the motor is moving, so a stopped motor is never sent to its My position by accident; set *My position* and the favourite button uses it.
- **Follow a Somfy remote:** the bridge learns a Situo or Telis remote's address (up to 3 per blind) and moves the position when that remote is used. Its presses also fire the *Remote button* event, with `somfy_remote` and `somfy_command`.
- **One receiver for both:** with any Somfy blind set up, the CC1101 listens half way between the A-OK frequency and 433.42 MHz with an 812 kHz bandwidth and decodes both protocols from the same signal. *Receive bandwidth* in the radio settings overrides this. Sending retunes to each motor's exact frequency.
- The Activity tab shows Somfy frames (button, remote address and rolling code).
- `POST /api/somfy` with `pair`, `learn`, `cancel`, `forget_remotes`, `resync` and `send` (raw up/down/my/prog, for testing).

### Notes
- Rolling codes are written to flash before every send and are part of the backup. Restoring a backup never moves a code backwards. *Resync* skips a code ahead by 20, only needed if a motor stops answering after restoring onto a new board.
- The ESPHome `aok_rf` component is still A-OK only. For Somfy in ESPHome use the firmware, or ESPSomfy-RTS.

## Unreleased


### ESPHome component
- `aok_rf` now has the bridge's radio features: `extra_remotes`, `on_long_press`, an `rssi:` sensor, `repeat_remote`, `aok_rf.frequency_scan`, and pairing to the component's own ID (`own_id`, `own_channels`, `aok_rf.pair`, `program:` in `remote:`).
- New `esphome/full-example.yaml` doing scenes, schedules with sunrise/sunset, named positions, a lock, a window interlock and wind protection in YAML. CI checks and compiles it. The table at the end of `docs/esphome.md` maps each bridge feature to its ESPHome version.

## [1.6.0] - 2026-11-17

### Added
- **Pair motors to the bridge's own ID (experimental):** the bridge learns your remote's P button when you press it, then *Pair automatically* sends P, P and Up from its own remote ID on a channel. That motor then keeps working from Home Assistant even if the hand remote is lost or replaced. Without the P button learned, press P twice on the hand remote and use *Pair now*. Not yet confirmed on every A-OK motor: if the motor doesn't jog at the end, nothing has changed.
- **Repeater:** *Repeat the hand remote* sends each press again from the bridge, for a motor at the edge of the remote's range.
- **Frequency scan:** hold a remote button and press *Scan*. The bridge measures 433.42–434.42 MHz and offers to tune to where your remote is strongest.
- **Tilt** for venetian blinds: set a *Tilt time* and Home Assistant gets a tilt slider (`<base>/<blind>/tilt/set`, or `TILT:<0-100>`).
- **Live updates:** the page gets the state pushed over a WebSocket (port 81) several times a second while blinds move, instead of polling. Polling stays as the fallback.
- **Ethernet build** for the WT32-ETH01 (`…-eth-factory.bin` / `…-eth-ota.bin`). *Check for updates* picks the right build.

## [1.5.0] - 2026-11-17

### Added
- **Scenes:** move several blinds to set positions in one go (up to 8). Each scene appears in Home Assistant as a scene, on the Blinds tab as a button, and over MQTT at `<base>/scene/set`.
- **Schedules on the bridge:** at a time, or at sunrise or sunset with an offset, on chosen days, for all blinds, one blind or a scene (up to 12). They keep working when Home Assistant or the internet is down. Set the location for sunrise and sunset; *Use this device's location* fills it in from a phone.
- **Named positions:** up to four per blind (for example *Privacy 30 %*), as buttons on the blind's menu and in Home Assistant, or `PRESET:<n>` over MQTT.
- **Lock** per blind (menu, Home Assistant switch, or `LOCK`/`UNLOCK`): Home Assistant, scenes, schedules and *All blinds* leave a locked blind alone. The hand remote and Stop still work.
- **Window interlock:** `<base>/<blind>/interlock` `ON` from a window or door sensor stops the blind closing (and stops it if it's closing).
- **Wind protection:** `<base>/wind` `ON` retracts every blind set to *Awning* and holds it there until `OFF`.
- **Other remotes:** *Add another remote* on the Remote tab follows a second hand remote or wall switch (up to 3), so positions stay right whichever remote is used.
- **Long presses:** the remote event fires `open_long`, `close_long` or `stop_long` when a button is held for 1.5 s or more.
- **Signal strength** of every received frame (Activity) and a *Remote signal* sensor in Home Assistant.
- **Any channels over MQTT:** `<base>/channels/set` with `1,3:OPEN` or `2-5:CLOSE`.
- **Per-blind availability** in Home Assistant: a blind shows unavailable while the radio isn't working or the remote isn't learned.
- **Safe mode:** after three crashes in a row the bridge starts without the radio and MQTT, so its page, logs and updates still work. It's back to normal after a restart once fixed.
- Scenes and schedules are included in the backup.

## [1.4.0] - 2026-11-17

### Security
- **Only the bridge's own page can change things.** Before, any website opened on a phone or computer on your network could send the bridge commands, or even new firmware, without you seeing it (cross-site requests). Every change now needs a header that only the bridge's page sends. Other sites can't add it.
- **PlatformIO over-the-air updates need the admin password.** With no password they're now off. Updates from the web page keep working.
- **Firmware uploads are checked as they arrive.** A file that isn't ESP32 firmware, a failed write or a cancelled upload stops straight away with a clear message in the log.
- A warning on the dashboard while no admin password is set.

### Fixed
- **Blinds went past their target when the MQTT broker was down.** The bridge froze for a few seconds each time it retried the broker. If that happened during a "go to 30 %" run, the Stop was sent late. Broker retries now wait until no blind is moving, back off to once a minute, and time out sooner. *Find broker* and *Test connection* wait for the blinds to stop too.
- **A neighbour's remote could wipe remote learning.** A learning session now sticks to the first remote it hears, and the page says when it ignored another one.

### Added
- **Safe reversing.** Asking a moving blind to go the other way now stops it, waits 0.7 s, then reverses, instead of flipping the motor's direction in one go.
- **Reference run.** Asking for a position like 40 % when the bridge doesn't know where the blind is now runs it to the nearer end first, then on to 40 %, instead of refusing.
- **Start-up delay** per blind (*Set up blinds*): the time the motor takes to start after a command. Short moves land closer to the target.

## [1.3.0] - 2026-11-14

### Added
- **Find broker** in *Settings → Home Assistant*: the bridge looks for MQTT brokers on your network (`_mqtt._tcp` services, and Home Assistant itself via `_home-assistant._tcp` or `homeassistant.local`), then checks that port 1883 answers. Click a result to fill in the address. It runs on its own when MQTT isn't set up yet.
- **Test connection**: tries the address, username and password on the page before you save. It says plainly whether the broker can't be reached or rejected the login.

## [1.2.0] - 2026-11-14

First public release.

### Added
- **Web installer** on GitHub Pages: flash from Chrome or Edge, then set Wi-Fi over USB (Improv).
- **Check for updates** in Settings: the page fetches the latest release from GitHub and installs it. The bridge itself doesn't need internet access.
- **ESPHome component** `aok_rf` for people who'd rather run ESPHome ([docs/esphome.md](docs/esphome.md)).
- **All blinds** bar: Open all / Stop / Close all in one radio frame. Also available over MQTT at `<base>/all/set`. *Not yet tested on real motors.*
- **Rooms**: blinds are grouped by room on the dashboard, and Home Assistant suggests that area.
- **Backup and restore** of blinds, remote codes, pins and settings (no passwords).
- **Blind brand** setting for the title, setup Wi-Fi name and Home Assistant names. It defaults to A-OK; upgraded Motolux bridges keep Motolux.
- **Time zone** setting, filled in automatically from the browser.
- MQTT discovery now includes an `origin` block linking to this project.

### Changed
- Frequent polls use a light state (`/api/state?lite=1`). Activity frames and the calibration log are fetched only when they change.
- Positions are saved to flash 3 s after blinds stop, not on every command, to save flash wear.
- The page is cached by the browser with an ETag, so repeat visits are instant.
- New devices default to `aok-blinds` / `aok_blinds` and a `homeassistant.local` broker. Upgraded devices keep their settings.

## [1.1.0] - 2026-11-14

### Added
- New dashboard:
  - **Blinds**: positions animate smoothly while moving, with a drawn window for each blind.
  - **Remote**: learning steps tick off, a channel-to-blind map, and a channel tester.
  - **Set up blinds**: collapsible editors with test buttons.
  - **Calibrate**: live timer, plus a calibration log kept on the device.
  - **Activity**: names buttons the bridge doesn't act on, and separates your remote from other remotes.
  - **Logs**: the last 120 events, downloadable.
  - **Settings**: everything in one place.
- Remembers which channels the remote has used, and offers to add blinds for them.
- NTP time for the log and the calibration history.

## [1.0.2] - 2026-11-14

### Changed
- Renamed to Motolux Blind Bridge in the page, setup Wi-Fi, hostname, MQTT topics and Home Assistant.

## [1.0.1] - 2026-11-14

### Added
- Status chips link to their tabs.
- Wi-Fi scan on the Network tab.

### Fixed
- Captive-portal probes from phones and Windows filled the log with "request handler not found". They're now answered directly.

## [1.0.0] - 2026-11-07

First version:
- Learns the hand remote on the device.
- Follows the remote live.
- Timed positions with separate up and down travel times.
- Groups, favourite position and calibration.
- MQTT discovery for Home Assistant.
- Web setup and a Wi-Fi setup portal.
