# Log and repository discovery

Discovery was performed 2026-07-21 before source changes. No repository was
initialized and no raw log was modified.

## Repository identity

- Supplied directory: `/home/ethan/year2_scf_drone` (not Git; contains only
  `training/` artifacts).
- Actual repository root: `/home/ethan/year1_scf_drone/micro-quad-SLAM`.
- Remote: `origin https://github.com/exie1122/micro-quad-SLAM.git` (fetch/push).
- Starting branch/commit: `experimental` at
  `19993ac8d3c4ac993f720660565cb583e442c0f2`.
- Upgrade branch: `autonomy-frontier-hardening`, created from that commit.
- Starting worktree: clean except ignored `python/__pycache__/`; it was not
  touched. No tags and no other local/remote historical branches were present.
- Log history: all committed `log/` evidence entered at `19993ac` (`Add experiment
  logs and scanlogs`). Git LFS is not installed/configured and no LFS pointer file
  or `.gitattributes` was found.

The non-Git parent `/home/ethan/year1_scf_drone` contains an audit/reorganization
copy with additional raw evidence. It is outside the repository but was included
in discovery because filenames and prior documentation refer to it.

## Binary scanlogs in the Git worktree

Every file below has an exact `SCLOG3\n` header and 642-byte packed `SCN3`
records. Timestamp ranges are host monotonic milliseconds, not wall-clock time.

| Path under `log/` | Bytes | Complete records | Timestamp range ms | Structure / likely session |
|---|---:|---:|---:|---|
| `3_frontierTest_scanlog.bin` | 48,799 | 76 | 50,317–65,018 | clean single frontier session |
| `3_hallWay_scanlog.bin` | 114,283 | 178 | 125,787–160,480 | clean single hallway session |
| `4_frontierTest_scanlog.bin` | 60,997 | 95 | 33,069–51,495 | clean single frontier session |
| `4_hallWay_scanlog.bin` | 248,461 | 387 | 27,390–160,480 | aligned, two concatenated sessions (one timestamp reset) |
| `5_frontierTest_scanlog.bin` | 85,393 | 133 | 23,467–44,206 | aligned, two concatenated sessions (one reset) |
| `7_frontierTest_scanlog.bin` | 78,331 | 122 | 76,947–123,597 | clean single frontier session |
| `changedEnvironment_scanlog.bin` | 153,824 | 234 | 22,884–123,597 | three sessions; one 4,231-byte corrupt/non-record gap; resync required |
| `frontierTest_scanlog.bin` | 45,589 | 71 | 42,639–56,360 | clean single frontier session |
| `logs/CLEAN_scanlog.bin` | 82,183 | 128 | 78,048–108,644 | clean single comparison session |
| `logs/mixed_scanlog.bin` | 1,220,608 | 1,901 | 23,471–419,909 | four concatenated sessions; final SCN3 record truncated to 159 bytes |
| `logs/scanlog.bin` | 156,655 | 244 | 75,415–123,044 | clean single session; **not** identical to the same-sized external `floor1_scanlog.bin` (different sha256 and timestamps) |
| `logs/veryClean_scanlog.bin` | 100,801 | 157 | 21,209–64,735 | clean single comparison session |

The upgraded C replay reader accepts these as SCLOG3, warns and resynchronizes on
the gap, reports timestamp resets, and warns on truncated tails. The old
`python/plot_scan.py` was hardened to do the same and now rejects other versions.

## Human-readable logs in the Git worktree

These are runtime text logs with bracketed monotonic seconds and state events:

| File(s) | Bytes / range | Notes |
|---|---|---|
| `2_hallWay_log.txt` | 161,707; 14.342–99.620 s | hallway session |
| `3_frontierTest_log.txt` | 162,891; 14.189–105.697 s | prelock timeout and landing evidence |
| `3_hallWay_log.txt` | 367,291; 14.225–186.556 s | hallway sessions |
| `4_frontierTest_log.txt` | 132,684; 14.194–87.687 s | frontier move, settle, then no-frontier land |
| `4_hallWay_log.txt` | 2,010,163; 14.225–845.725 s | multi-session hallway; byte-identical to `result/4_hallWay_log.txt` |
| `5_frontierTest_log.txt` | 219,429; 14.193–91.348 s | takeoff ramp/assist and recovery failures |
| `5_hallWay_log.txt` | 239,880; 14.190–113.485 s | hallway session |
| `6_frontierTest_log.txt` | 130,037; 14.188–83.661 s | frontier session; no paired SCLOG3 in Git |
| `7_frontierTest_log.txt` | 301,906; 14.281–164.308 s | no-progress retry and no-frontier land |
| `changedEnvironment_log.txt` | 866,412; 14.281–273.747 s | changed-environment multi-session |
| `frontierTest_log.txt` | 176,698; 14.190–99.102 s | frontier session |
| `frontier_session_75878ms_log.txt`, `log.txt` | 1,287,633 each; 14.226–339.680 s | exact duplicates (SHA-256 identical) |
| `hallWay_log.txt` | 140,104; 14.201–77.392 s | hallway session |
| `logs/log.txt` | 422,104; 14.189–224.359 s | comparison-session runtime log |
| `session_30951ms_5_frontierTest_log.txt` | 69,064; 14.410–46.132 s | extracted session |
| `session_32736ms_4_frontierTest_log.txt` | 132,684; 14.194–87.687 s | exact duplicate of `4_frontierTest_log.txt` |

`FC STATUSTEXT ... crash_dump.bin` lines are preserved as flight-controller
messages; they are not proof that the companion process itself crashed.

## Additional evidence outside the Git worktree

- `/home/ethan/year1_scf_drone/data/scanlogs/` duplicates the named committed
  SCLOG3 files and additionally contains:
  - `2_frontierTest_scanlog.bin`: 69,580 bytes; header at byte 0, first record at
    byte 4,096; 102 complete SCN3 records followed by no partial record. The
    4,089-byte zero/non-record prefix makes the old fixed-stride parser
    incompatible; hardened replay resynchronizes.
  - `floor1_scanlog.bin`: 156,655 bytes, 244 clean SCN3 records; despite the same
    size, it is a different session from `log/logs/scanlog.bin` in Git.
- `/home/ethan/year1_scf_drone/logs/scanlog_from_drone.bin`: exactly seven bytes,
  only `SCLOG3\n`, zero frames.
- `/home/ethan/year1_scf_drone/logs/:USERPROFILE\Downloads\scanlog.bin`:
  208,564,224 bytes, explicit `SCLOG1\n`/`SCN1` format, at least 1,360 record
  markers (nominal early stride 556 bytes) plus sparse/gapped content. It is not
  SCLOG3 and is intentionally rejected by all SCLOG3 tooling.
- `/home/ethan/year1_scf_drone/data/bin/*.bin`: four ArduPilot DataFlash logs,
  identified by `A3 95 ... FMT` magic, sizes 1,145,060; 2,720,256; 1,279,046;
  and 1,240,576 bytes. They are not SCLOG files and require ArduPilot tooling.
- `docs/logs/` holds copies/extracts of the named text sessions plus
  `2026-04-06_frontier_log.txt`, `session_41661ms_frontierTest_log_arm_window.txt`,
  and a DataFlash session summary. `logs/*.txt` holds older hover/exploration
  text logs. These are retained outside Git.
- `data/csv/` and `data/json/` contain derived trajectories, exported SCLOG frame
  windows, and room metadata. They are outputs, not substitutes for raw inputs.
- `.npz` files under `models/` are YOLO model tensors, not flight logs.

## Figures and provenance

Ten external analysis PNGs were subsequently regenerated byte-for-byte from
`logs/CLEAN_scanlog.bin` and `logs/veryClean_scanlog.bin`; the startup-filter
comparison is a high-confidence 157/128-frame match but is not byte-identical
with the current plotting code. The specific raw input
for committed `assets/Figure_1.png`, `mapped_environment_2.png`, and
`image (9).png` is not recorded, so no exact figure-to-log claim is made. Exported
JSON filenames directly tie `4_frontierTest` and `4_hallWay` to those sessions.

## Discovery verdict

The previous blanket conclusion that the project was “not in Git” and that logs
were missing was **incorrect**: the Git worktree is nested and substantial raw
SCLOG3 plus DataFlash evidence exists. The figure inputs formerly referenced as
`/home/ethan/scanlog.bin` and `/home/ethan/CLEAN_scanlog.bin` are now matched to
the two archived files above. The large archived SCLOG1 file remains unrelated
and must not be substituted.
