# Setting up the bridge

## 1. Flash

Use the [web installer](https://olliemaller.github.io/aok-blind-bridge/) (Chrome or Edge on a computer). It flashes the firmware, then asks for your Wi-Fi over USB. If the installer can't connect, hold **BOOT** on the ESP32 while you press Install.

If you skip the Wi-Fi step, the bridge opens a setup network called **AOK-Blinds-xxxx**. Join it and the setup page opens. If it doesn't, browse to `http://192.168.4.1`. Then go to **Settings → Wi-Fi**, press **Scan**, pick your network and save. The bridge restarts and joins it.

## 2. Open the dashboard

Browse to `http://aok-blinds.local`, or the IP address your router shows for it. The pills at the top show Wi-Fi, Home Assistant, the radio and the remote. Click one to jump to its settings.

If **Radio** is red, the ESP32 can't talk to the CC1101. Check that VCC goes to 3V3, then check SCK, MISO, MOSI and CSN. MISO and MOSI swapped is the usual mistake.

## 3. Learn the remote

**Remote → Start learning.** Hold the remote near the bridge and press **Up**, then **Down**, then **Stop** (the middle button), on any channel. Each step ticks off as it's heard.

**Channels and blinds** then shows every channel you've used on the remote and the blind it's linked to. Use each channel once to fill it in. **Test a channel** sends a command on any channel, so you can see which blind moves.

## 4. Set up the blinds

**Set up blinds → Add blind**, or press **Add** in the banner to add every channel the remote has used. For each blind:

- **Name** and **Room**. The room becomes its area in Home Assistant.
- **Remote channel**. Pick more than one to make a group that moves together.
- **Opening and closing time.** The Calibrate tab measures these.
- **Named positions.** Up to four, each with a name and %, shown as buttons on the blind and in Home Assistant.
- **Tilt time.** For venetian blinds: how long the slats take to turn from closed to open. Adds a tilt slider in Home Assistant.
- **Start-up delay.** How long the motor takes to start after a command (usually 0.3–1 s). Set it if short moves stop short of the target.
- **Favourite position** (optional). The ★ button sends the blind there.
- **Reversed.** Tick it if Up closes this blind.

Use the test buttons on each blind to check it's the right one, then **Save blinds**.

## 5. Calibrate

**Calibrate**, pick a blind, and make sure it's fully open:

1. Press **Start closing**. Press **It's stopped** the moment it stops.
2. Press **Start opening**. Press **It's stopped** when it reaches the top.

Each measurement is saved and added to the calibration log. Do it again any time a blind seems out.

## 6. Connect Home Assistant

**Settings → Home Assistant**: enter the broker (for the Mosquitto add-on, `homeassistant.local` or your Home Assistant's IP, port 1883) and an MQTT user. The card says *Connected*, or explains what's wrong. The blinds then appear under **Settings → Devices & services → MQTT**. See [home-assistant.md](home-assistant.md).

## Other settings

- **Blind brand.** Sets the name used in the title, the setup Wi-Fi and Home Assistant, for example `Motolux`.
- **Time zone.** Set automatically from your browser the first time. It's used for log and calibration times.
- **Web page password.** Adds a login (user `admin`) to the page. Set one: until you do, the dashboard shows a warning, and over-the-air updates from PlatformIO (`espota`) stay off. Updates from the page itself always work.
- **Backup.** Downloads your blinds, remote codes, pins and settings (no passwords). **Restore from file** loads them onto another board.
- **Logs.** The last 120 events: commands, remote presses, connections, calibrations and errors. Download them when reporting a problem.

## Scenes and schedules

**Scenes & schedules** has:

- **Scenes:** tick the blinds and their positions. *Run* tries one. Up to 8.
- **Schedules:** days, a time or sunrise/sunset with an offset in minutes, which blinds (all, one blind or a scene), and what to do (open, close, stop or a position). Up to 12. They run on the bridge, so they keep working if Home Assistant is down. They need the time zone (set in Settings) and, for sunrise/sunset, the location.
- **Location:** latitude and longitude, or *Use this device's location* on a phone.

## Remote tab extras

- **Pair motors to the bridge (experimental):** gives a channel the bridge's own remote ID, so the motor keeps working from Home Assistant if the hand remote is lost. Press **P** on the hand remote once so the bridge learns it, then **Pair automatically** (the motor jogs three times). Or press P twice on the hand remote yourself and use **Pair now**.
- **Repeat the hand remote:** the bridge sends every press again, for a motor at the edge of the remote's range.
- **Frequency scan:** hold a remote button and press **Scan**. The bridge measures 433.42–434.42 MHz and offers to tune to where your remote is strongest.

## Ethernet (WT32-ETH01)

The release has an Ethernet build, `aok-blind-bridge-<version>-eth-factory.bin`, for the WT32-ETH01 (ESP32 + LAN8720). Wire the CC1101 to: SCK 14, MISO 35, MOSI 15, CSN 4, GDO0 32, GDO2 33. The bridge uses the cable; Wi-Fi is optional, and the setup Wi-Fi only opens if the cable has no network for 30 s. *Check for updates* picks the Ethernet build automatically.

## Live updates

Open pages get the blinds' state pushed over a WebSocket on port 81 several times a second while anything moves, so positions and the activity update straight away. If port 81 is blocked, the page falls back to polling as before.
