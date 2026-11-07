# A-OK Blind Bridge (ESP32 + CC1101)

Local, cloud-free control of **A-OK 433.92 MHz blind motors** (AC127 remotes, AC520 RF Smart Hub family) from Home Assistant.
It's built on the protocol work in [pgnehm/ESP32-Orvibo-A-OK-HomeAssistant-Blind-Bridge](https://github.com/pgnehm/ESP32-Orvibo-A-OK-HomeAssistant-Blind-Bridge), and adds:

| Feature | What it does |
|---|---|
| Learn from your remote | Press Up, Down and Stop once. The bridge decodes the A-OK frames on the device, so there are no Python scripts and no recompiling. |
| Follows the hand remote | When someone uses the AC127 remote, the bridge hears it and moves the position in Home Assistant to match. |
| Position control | Each blind is a Home Assistant cover with a 0–100 % slider. Intermediate positions are timed, then the bridge sends Stop. |
| Separate up and down times | Many motors close faster than they open, so each direction has its own travel time. |
| Calibration helper | Time a full open or close from the web page and save it. |
| Position memory | Positions survive restarts and power cuts. |
| Favourite position | One tap from the web page or a Home Assistant button sends the blind to your favourite %. |
| Groups | A blind on several channels (for example `1,2,3`) moves them all with one command, and shows the average position. |
| Remote button event | A Home Assistant `event` entity fires on every hand-remote press, so the remote can also run automations. |
| Mark open / closed | Sets the position without moving the blind. Use it if something gets out of step. |
| Web setup | Wi-Fi setup portal, MQTT, blinds, radio pins and firmware upload, all in the browser. |

## Wiring (HW-395 / ESP32 DevKit, 38 pins)

Power the CC1101 from **3V3 only**.

| CC1101 pin | Signal | ESP32 pin |
|---|---|---|
| 1 | GND | GND |
| 2 | VCC | 3V3 |
| 3 | GDO0 (transmit) | P12 |
| 4 | CSN | P5 |
| 5 | SCK | P18 |
| 6 | MOSI | P23 |
| 7 | MISO | P19 |
| 8 | GDO2 (receive) | P27 |

GPIO 12 is an ESP32 boot-strap pin. If the board ever fails to start with the CC1101 connected, move GDO0 to **P4** and set GDO0 = 4 under *Radio & firmware*.

## Flashing

1. Open https://espressif.github.io/esptool-js/ in Chrome and click **Connect** (hold BOOT if it hangs).
2. Flash `aok-blind-bridge-hw395-factory.bin` at address **0x0**, with **Erase flash** ticked the first time.
3. Press EN/reset. Later updates use `aok-blind-bridge-hw395-ota.bin` under *Radio & firmware → Upload firmware*.

## First-time setup

1. Join the Wi-Fi network **AOK-Blinds-xxxx**. The setup page opens; if it doesn't, browse to `http://192.168.4.1`.
2. Under **Network**, enter your Wi-Fi details and the MQTT broker (Home Assistant's IP, port 1883, and an MQTT user). Save. The bridge restarts and joins your Wi-Fi.
3. Open `http://aok-blinds.local`, or the IP shown in your router.
4. Under **Remote**, press **Start learning**, then press **Up**, **Down** and **Stop** on any channel of the hand remote.
5. Under **Set up blinds**, add one row per blind with its remote channel. Add a row with several channels for a group.
6. Under **Calibrate**, time each blind closing and opening.
7. In Home Assistant, the blinds appear automatically under **Settings → Devices → MQTT**.

## Home Assistant entities

- `cover.<blind>`: open, close, stop, position
- Buttons per blind: **Favourite position**, **Mark as open**, **Mark as closed**
- `event.a_ok_blind_bridge_remote_button`: event types `open`, `close` and `stop`, with the `channels` attribute
- Diagnostics: Wi-Fi signal, uptime, IP address, last remote press, CC1101 radio, and a **Learn remote** button

MQTT topics use the base `aok_blinds/`: `<id>/set` (`OPEN` `CLOSE` `STOP` `FAVOURITE` `SYNC_OPEN` `SYNC_CLOSE` `0-100`), `<id>/state`, `<id>/position` and `remote/event`.

## Building from source

```bash
pio run -e hw395            # or esp32s3 for the upstream ESP32-S3 board
pio run -e hw395 -t upload
g++ -std=c++17 -I include test/test_protocol.cpp -o /tmp/aok_test && /tmp/aok_test
```

The protocol encoder is round-trip tested on the host (encode, add timing jitter, decode).

## Limits

- The motors don't report their position. Positions are estimated from travel time, so use **Mark as open/closed** if a blind is moved by hand.
- It only handles A-OK's 64-bit "new PWM" remotes. Older AC114-style remotes are decoded in the Activity log but can't be sent yet.
