# Software update over the internet

*2026-10-27. Same tool and shape as Horizon's FAAM.UI (`komcat/Horizon`,
`docs/software-update-design.md`), in C++.*

A station is installed ONCE by the release's `Setup.exe` into `C:\FireDog`;
every later release arrives through **Help > Version / Update**. The packages
are [Velopack](https://velopack.io) 1.2.161, published on this repository's
GitHub Releases next to the zip. The repository is public, so no token.

## On a station

```
C:\FireDog\                Install-FireDog.cmd = Setup.exe --installto C:\FireDog
  Update.exe               Velopack's updater - runs only while an update installs
  FireDog.ini              where the machine root is (core/Paths) - OUTSIDE current\
  current\                 FireDog.exe, FireDogRig.exe, every DLL, assets\,
                           config\scripts\, and the zip's seed config\ + products\
  packages\                the last downloaded package (the base for the next delta)
```

- **`current\` is replaced whole** by every update. That is safe only because
  nothing runtime-written lives beside the exe (`docs/machine-root-design.md`) -
  with one exception, `FireDog.ini`, which is why `InitPaths` looks for it one
  level up when the exe sits in `current\` beside an `Update.exe`. Anything a
  future change writes beside the exe is lost on the next update.
- `current\FireDog.exe` keeps its path across versions, so a per-exe firewall
  rule (the GigE camera) is added once per install.
- The seed in `current\` (one machine's `config\` and `products\`, see
  `package-release.ps1`) is read only by the first-start migration into an EMPTY
  machine root. On a station that already has its root it is inert, so an
  update can carry it.
- `Setup.exe` run on its own installs to `%LOCALAPPDATA%\FireDog`. vpk has no
  option to bake a default path in, so the path travels in `Install-FireDog.cmd`.
  `FireDog-win-Portable.zip` unzipped into a folder is the same layout and
  updates the same way.
- A copy NOT installed this way (a build tree, the hand-unzipped release zip)
  cannot update; the Version page says so and offers no button.

## Flow

1. **Check.** `App::Run` starts the updater after the hardware bring-up; it
   locates the install (no network) and queues one quiet check. A station with
   no internet logs `check failed` at Info and carries on. A found release
   shows in the Help menu label. The page's **Check now** repeats it.
2. **Download.** Velopack fetches the delta when the previous package is in
   `packages\`, else the full package, and verifies the SHA from the feed.
   Nothing that runs changes, so it is allowed mid-run.
3. **Restart and install.** Refused while `orch_.Busy()` or while any editor
   has unsaved edits (it closes without the exit confirm). It marks the update
   and closes FireDog through the normal exit. `main()` destroys `App` - which
   disconnects every device - and only THEN calls `AppUpdater::ApplyAtExit`,
   which starts `Update.exe`. Update.exe waits (60 s at most, hence after the
   teardown) for the process to end, replaces `current\`, starts the new exe.

A downloaded update is never applied silently: `RunStartupHook` sets
`SetAutoApplyOnStartup(false)`, and a plain exit after a download installs
nothing. Only Restart does.

## Rules

- **`update/AppUpdater.cpp` is the only TU that includes a Velopack header**,
  and only the FireDog client links it (CMake: `velopack_libc`, the prebuilt
  DLL copied beside the exe). FireDogRig has no business updating anything.
- **`AppUpdater::RunStartupHook` is the first line of `main`**, before
  `InitPaths`: during Setup and updates the exe is started with a hook argument
  and exits there.
- **`main()` owns the updater, not `App`**: the update must be handed over after
  every device is closed, which is after `App` is gone.
- **One worker, joined in the destructor.** Velopack's download cannot be
  cancelled, so an exit mid-download waits for it (the exit confirm says so).
- **The vpk version in `tools/release/pack-velopack.ps1` equals the
  `velopack_libc` version in `CMakeLists.txt`.** Bump both together.
- **Every release is a strictly newer SemVer** - `vX.Y` packs as `X.Y.0`.
  Velopack offers nothing that is not newer.
- The Version page is Admin (the `MinRoleFor` default): its button replaces
  the program.

## Code

| Piece | Where |
|---|---|
| Startup hook, owning the updater, apply after `~App` | `FireDog.cpp` |
| Check / download / apply-at-exit, the worker | `src/update/AppUpdater.{h,cpp}` |
| Version page: build info, status, Check / Download / Restart, notes | `src/ui/pages/PageVersion.cpp` |
| Help menu entry (with the "available" hint), exit-confirm line | `src/ui/pages/MainWindow.cpp` |
| `FireDog.ini` above `current\` | `src/core/Paths.cpp` (`IniFolder`) |
| Packing + upload | `tools/release/pack-velopack.ps1`, the `/release` skill |

## Releasing

`/release` does it. After `package-release.ps1`:

1. `pack-velopack.ps1 -Version vX.Y -NotesFile <notes>` unzips the release zip
   (so an installed station runs exactly the zip's files), downloads the
   previous full package for the delta, runs `vpk pack` with
   `--framework vcredist143-x64` (Setup installs the VC++ runtime a new PC
   lacks), and writes `Install-FireDog.cmd`.
2. After `gh release create`: `pack-velopack.ps1 -Version vX.Y -Upload` merges
   the packages into the same release and adds `Install-FireDog.cmd`.

Packages are unsigned: Windows SmartScreen warns on a station's first Setup.

## First install on an existing station

A station running an unzipped release moves over once: download
`FireDog-win-Setup.exe` and `Install-FireDog.cmd` into one folder and run the
`.cmd`. Copy the old folder's `FireDog.ini` (if it has one) to
`C:\FireDog\FireDog.ini`. The machine root carries over untouched - the new
install reads the same `D:\ProgramData\FireDog`. Re-add the camera's firewall
rule for `C:\FireDog\current\FireDog.exe`.
