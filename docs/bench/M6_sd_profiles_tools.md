# M6 test: SD profiles and PC tools

**Goal (plan section 11, M6):** the tool self-test recovers known parameters within 5 %. A generated profile replays from SD and matches its source torque trace in `0x4B0`.

## 1. PC tools (no hardware)

```text
cd tools
python -m pip install -r requirements.txt
python selftest.py
```

It should finish with `21/21 checks passed`. It needs `g++` on `PATH`; on Windows, WinLibs or MSYS2 MinGW both work. The run here gave the following:
- `J`, `Tc`, `b` and `c` recovered to within 0.11 %;
- a synthetic baler load recovered with an RMS error of 1.8 Nm on a 150 Nm peak;
- the profile replayed through the firmware's profile code to within 0.05 Nm (storage is 0.1 Nm).

## 2. SD card

1. Format a microSD card FAT32 and copy `sd_card/presets` and `sd_card/profiles` to its root.
2. Make a test profile from the self-test's synthetic baler log, or from any bench log with M5 streaming:
   ```text
   python selftest.py --keep st
   copy st\baler_profile.csv ..\sd_card\profiles\   (and onto the card)
   ```
3. Insert the card and reset the LilyGo. The banner shows `SD card found`.

## 3. Replay on the bench

With the Zombie in Run mode and SavvyCAN logging:

1. `profile list` shows the files on the card.
2. `profile load baler_profile`. The reply gives the rows and the load time. The row count must match the file.
3. `preset noload`, then `profile start`:
   - the LED turns yellow;
   - SavvyCAN shows a `ProfileStart` event;
   - `SIM_Slow_4B1.ProfileState` = Running.
4. When it ends: a `ProfileStop` event, and `ProfileState` = Finished. With `profile loop 1` it repeats instead.
5. **Match check.** Export the capture and compare `SIM_Fast_4B0.TorqueLoad` against the profile file. Subtract the base friction (`Tc` + `b`|ω|) to get the profile part. It should follow the file row for row, one tick late at most.
6. Try `profile load` while running: it must reply `err stop the profile first`.
7. Preset override: `preset baler` now uses `/presets/baler.json` from the card. `stat` shows `J=0.900`.

## 4. Errors to try

| Command | Expected reply |
| --- | --- |
| `profile load nosuch` | `err file not found` |
| A CSV with a row at 15 ms | `err rows must be 10 ms apart from 0 at line N` |
| `preset nosuch` | `err unknown preset, try presets` |

Send back: a SavvyCAN capture of the replay, and the `profile load` reply line.
