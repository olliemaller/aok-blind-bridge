# The A-OK 433 MHz remote protocol

These are notes on the 64-bit A-OK remote format, as used by the AC127 remotes that come with Motolux blinds and others. The format and checksum rules were first worked out by pgnehm ([ESP32-Orvibo-A-OK-HomeAssistant-Blind-Bridge](https://github.com/pgnehm/ESP32-Orvibo-A-OK-HomeAssistant-Blind-Bridge)). The rest of this page comes from decoding a real remote with this bridge. The code is in [`include/aok_protocol.h`](../include/aok_protocol.h).

## Radio

- 433.92 MHz, on-off keying (OOK).
- Pulse-width coded, starting high. The CC1101 runs in asynchronous serial mode: GDO0 is the transmit data input, and GDO2 outputs the received data.

## One frame

| Part | Level and length |
|---|---|
| Sync | high **4960 µs**, low **670 µs** |
| 64 data bits | each bit is one high and one low pulse |
| 2 trailer bits | `1`, then `0` |
| Gap before the next repeat | low about **9.8 ms** |

| Bit | High | Low |
|---|---|---|
| `0` | long, 582 µs | short, 263 µs |
| `1` | short, 263 µs | long, 582 µs |

The remote sends each press about 8 times. The bridge does the same. The decoder accepts 140–420 µs as short, 421–950 µs as long and 3.8–6.5 ms as the sync high, which covers the drift between remotes and the jitter of an ESP32 interrupt.

## The 64 bits

```
 63           40 39      32 31            16 15      8 7       0
+---------------+----------+----------------+---------+---------+
|  prefix (24)  | remote ID|  channel field |  button | check   |
+---------------+----------+----------------+---------+---------+
```

- **Prefix (24 bits)**: fixed for a remote.
- **Remote ID (8 bits)**: fixed for a remote. The prefix and ID together are what the motor was paired with.
- **Channel field (16 bits)**: which channels the press is for. It starts as `0xFFFF`. For each channel *n* (1–16), bit `(n + 7) mod 16` is cleared:

  | Channel | 1 | 2 | … | 8 | 9 | … | 16 |
  |---|---|---|---|---|---|---|---|
  | Bit cleared | 8 | 9 | … | 15 | 0 | … | 7 |

  Clearing several bits addresses several channels in one frame. The bridge uses this for groups and for *All blinds*.
- **Button (8 bits)**:

  | Byte | Button | Notes |
  |---|---|---|
  | `F4` | Up | the bitwise NOT of `0B` |
  | `BC` | Down | NOT `43` |
  | `DC` | Stop | NOT `23` |
  | `AC` | Programming (P) button, probably | NOT `53`. Seen in the Activity tab; the bridge doesn't act on it. |
  | `DB` | Button released, probably | NOT `24`. Seen after holding Up or Down. |

  Anything else shows as *Other button* with its code. Please report what button made it.
- **Check (8 bits)**: depends on the remote ID and the channels, so each channel's frame has a different last byte:

  ```
  offset = signed8( 2 + Σ over selected channels of 2^((channel − 1) mod 8) )
  check  = (base + remote_id − offset) mod 256
  ```

  `signed8` wraps a value into −128…127. The **base** is the same for every channel of one button on one remote. So one captured press of each button is enough to build frames for any channel or mix of channels:

  ```
  base = (captured_check − remote_id + offset_of_captured_channels) mod 256
  ```

## What "learning" stores

For each of Up, Down and Stop, the bridge keeps the 16-bit `button<<8 | base` value. With the prefix and remote ID, that's everything needed to send any button on any channel. A backup file has these under `remote`. The ESPHome component takes the same values in its `remote:` block.

## Open questions

- Older A-OK remotes (AC114 and similar) use a shorter format. The decoder ignores them.
- **Pairing to the bridge's own ID (experimental, 1.6.0).** The bridge learns the P button (`AC`) when it's pressed on the hand remote. *Pair automatically* then sends P twice on the channel from the hand remote's ID (the usual "P, P" that puts an A-OK motor in pairing mode) and Up from the bridge's own ID: the same prefix and button bases with a different remote ID. Whether a motor accepts a new ID built this way hasn't been confirmed on every motor yet. Limit setting isn't implemented: set limits with the original remote.
- If your remote's frames don't decode, the Activity tab and the log will still show what the radio picks up. Please open an issue with them.

## Somfy RTS (since 1.7.0)

The bridge also speaks Somfy RTS at 433.42 MHz (`include/somfy_rts.h`, tested by `test/test_somfy.cpp`). A frame is 7 bytes: `0xA0 | key`, `command << 4 | checksum`, a 16-bit rolling code and a 24-bit address. The checksum makes the XOR of all 14 nibbles zero; each byte is then XORed with the byte sent before it. Bits are Manchester coded with 640 µs halves (a 1 is low then high). The first frame has a wake-up pulse and 2 hardware syncs (2560 µs high and low); repeats have 7. A 4550 µs software sync comes before the data and a 30 ms gap after it. Commands: My 1, Up 2, Down 4, Prog 8.

Each Somfy blind has its own address; the motor learns it like an extra remote when it's in programming mode and receives Prog. The motor only acts on codes higher than the last one it saw from that address (within about 100), so the bridge saves the next code before every send.

Receiving both protocols with one CC1101: with a Somfy blind set up, the receiver centres on the midpoint of the A-OK frequency and 433.42 MHz with an 812 kHz bandwidth (both are ±250 kHz from the centre). The edge stream from GDO2 goes to both decoders. Sending retunes to the exact frequency, then back.
