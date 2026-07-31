# FIFO, Interrupts, Timestamp, And Embedded Engines

> Source confidence: vendor fact from the datasheet and AN5130. Library status:
> chip-only/unsupported except for passive diagnostics and the explicitly
> destructive bounded FIFO purge.

## FIFO Data Model

- Capacity is 4 KiB, addressed as up to 2048 16-bit words.
- FIFO data have no type tags. `FIFO_PATTERN[9:0]` in `FIFO_STATUS3/4` identifies
  the next word's position in the configured repeating pattern.
- Data set 1 is gyro XYZ; data set 2 is accel XYZ; data set 3 is
  `SENSORHUB1..6`; data set 4 is `SENSORHUB7..12` or a selected special source.
- `CTRL3_C.IF_INC=1` and `BDU=1` are required for the vendor FIFO procedures.
- Threshold `FTH[10:0]` is in 16-bit words, range 0..2047.

Source: datasheet pp. 34-38, 54-58, 79-81; AN5130 pp. 74-95.

## FIFO Modes

| Encoding | Mode | Exact behavior |
|---:|---|---|
| `000` | Bypass | FIFO disabled/cleared. |
| `001` | FIFO | Collect until full, then stop. |
| `011` | Continuous-to-FIFO | Overwrite oldest data before the trigger; on the trigger, transition to FIFO mode, continue filling if space remains, then stop when full. |
| `100` | Bypass-to-continuous | Remain bypassed until the first trigger edge, then continuously overwrite oldest data until software selects Bypass. |
| `110` | Continuous | Collect continuously and overwrite oldest data when full. |

Values `010`, `101`, and `111` are reserved. Source: datasheet p. 58;
AN5130 pp. 80-86.

## Trigger, Status, And Drain Rules

- With the normal internal trigger, effective FIFO cadence is
  `min(max(ODR_XL, ODR_G), ODR_FIFO)`. Sensor-hub end-of-operation or step
  detection can be selected instead. Source: AN5130 Figure 32, p. 87.
- `DIFF_FIFO[10:0]` reports unread 16-bit words. During overrun it can read zero;
  zero is not empty unless `FIFO_EMPTY=1`.
- `FIFO_FULL_SMART` predicts whether the next complete stored data set will
  make the FIFO full, not necessarily the next individual word.
- Read status and pattern before consuming `FIFO_DATA_OUT_L/H`. Never read data
  while `FIFO_EMPTY=1`, and never change ODR/decimation/FIFO configuration while
  unread data remain. Drain, select Bypass, reconfigure, then select the target
  mode. Source: AN5130 pp. 79, 83, 87-93.
- `ONLY_HIGH_DATA` is valid only with `DEC_FIFO_G != 000` and
  `DEC_FIFO_XL = 000`; it packs accel/gyro high bytes in a special layout.
  Setting the bit alone is not a complete recipe. Source: AN5130 p. 93.

Because the stream is untagged and profile-dependent, this library does not
claim typed FIFO acquisition. `startFifoPurge()` intentionally discards at most
the caller's bounded word limit and reports overrun/truncation evidence.

## Interrupt Behavior

INT1 and INT2 polarity, drive type, latching, and route selection are separate
concerns. Pulse duration is event-specific; there is no safe universal rule
that every nonlatched interrupt lasts 75 us or one ODR period.

- Pedometer/embedded pin pulses are approximately 75 us.
- Activity/inactivity uses `1/ODR_XL` behavior.
- Free-fall follows the active condition.
- Tap recognition follows its Shock/Quiet/Duration windows.

Interrupt generation uses accelerometer data, so the accelerometer must be
active for free-fall, wake-up, 6D/4D, tap, and activity/inactivity. Source:
AN5130 sections 5 and 6, pp. 32-57.

## Pedometer And Timestamp Dependencies

- Embedded functions run from a 26 Hz accelerometer-derived engine and require
  an active accelerometer at least at 26 Hz.
- Step-delta timing requires `TIMER_EN=1` and `TIMER_HR=0`; only then is one
  `STEP_COUNT_DELTA` LSB 1.6384 s.
- Timestamp is a 24-bit counter. Writing `0xAA` to `TIMESTAMP2_REG` resets it
  and clears the end-counter interrupt. Select timestamp resolution before
  enabling it. Source: AN5130 pp. 50-57.

## Sensor Hub Dependencies

- Internal triggering requires the accelerometer active and is limited to
  104 Hz.
- External INT2 triggering still requires at least one internal sensor active.
- Configure an external sensor with the internal trigger before switching to
  external trigger mode.
- Up to four external sensors may be scheduled. Their data occupy
  `SENSORHUB1..18`; NACK flags are exposed in `FUNC_SRC2`.

Source: datasheet sensor-hub registers pp. 69, 76-85, 99-104; AN5130
pp. 58-73.
