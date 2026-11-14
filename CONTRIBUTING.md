# Contributing

Issues and pull requests are welcome, especially from people with other A-OK remotes or motors.

## Reporting a remote that doesn't work

Open an issue that includes:

- **The remote's model**, usually on the back (for example AC127-06L), and the blind brand.
- **Its frames.** Go to the *Activity* tab and press each button once. Copy the rows, or use *Logs → Download*.
- **What happened** compared with what you expected.

## Working on the code

- **Firmware:** `src/main.cpp`. Build with `pio run -e hw395`.
- **Dashboard:** `ui/index.html`. Run `python3 tools/mock_bridge.py` and open http://localhost:8099 to work on it without hardware. The build gzips it into the firmware.
- **Protocol:** `include/aok_protocol.h`. Keep `esphome/components/aok_rf/aok_protocol.h` identical; CI checks this. Run the tests with `g++ -std=c++17 -I include test/test_protocol.cpp -o /tmp/t && /tmp/t`.
- **ESPHome:** `esphome compile esphome/example.yaml`.

## Releasing

1. Bump `FW_VERSION` in `src/main.cpp`.
2. Add a `## [x.y.z] - date` section at the top of `CHANGELOG.md`.
3. Push to `main`.

The release workflow then builds the firmware, publishes a GitHub release with the factory and OTA images, and updates the web installer and the update manifest. In each repository, set **Settings → Pages → Source** to **GitHub Actions** once.
