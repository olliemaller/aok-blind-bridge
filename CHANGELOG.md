# Changelog

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
