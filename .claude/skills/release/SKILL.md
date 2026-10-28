---
name: release
description: Cut a FireDog release - configure the x64-release-deploy preset, get a fresh build, pack the deploy zip with tools/release/package-release.ps1, write release notes from the commits since the last tag, tag vX.Y, and publish a GitHub release with the zip attached. Use this whenever the user asks to release, publish, deploy, package, ship, tag a version, "make a build for the machine", "put it on the release page", or attach an exe/zip to GitHub - even if they only say "v0.3" or "release it" with no other words.
---

# Release

The deliverable is a GitHub release at `github.com/komcat/FireDog/releases` with
the zip `FireDog-vX.Y-x64-release-deploy.zip`, the Velopack update packages
(`FireDog-win-Setup.exe`, `Install-FireDog.cmd`, `FireDog-win-Portable.zip`,
the `.nupkg`s, `releases.win.json` - what installed stations update from, see
docs/software-update-design.md), and notes by area. Everything below exists
because a step was skipped or done wrong once; the order matters.

## 0. Preconditions

- `git status` clean and master pushed (`git status -sb` says nothing about
  "ahead"). Push first if not: the tag must point at a commit GitHub has.
- `gh auth status` shows the komcat account. It is logged in on this machine.
- Decide the version. `gh release list` and `git tag --sort=-v:refname` show
  what exists (v0.1 released 2026-08-13, v0.11 tag never published, v0.2
  released 2026-09-23). Propose the next `vX.Y` and use it unless the user
  names one; if they already named it, do not ask.

## 1. Configure the deploy preset - from the VS 18 environment

```
$vc = "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"
cmd /c "`"$vc`" >nul 2>&1 && cmake --preset x64-release-deploy"
```

Visual Studio 18 (with its bundled CMake 4.3.1) is the toolchain every build
on this machine uses. VS 2022 is also installed and looks the same, but its
older CMake lacks the `SegmentHeap.cmake` the presets include and then fails
with a misleading "Language 'CXX' is currently being enabled" from the deps.
Configuring is allowed (CLAUDE.md permits `cmake --preset`); it re-globs
sources so a build sees every file added since the last configure. Expect
"HALCON SDK found", "PI GCS2 SDK found" etc. in the tail.

## 2. Build it yourself - then VERIFY it landed

**Inside `/release`, Claude runs the deploy build.** The user asked for this on
2026-10-08: the release is automated end to end. It is the ONE exception to
the project rule that the user compiles - a normal session still never runs
`cmake --build`; outside this skill, hand the user the command.

**Tag first, locally (do NOT push yet).** The exe reports `git describe`
(core/BuildInfo, re-read on every build), so the tag must exist on the commit
being built or the exe calls itself `v<previous>-N-g<hash>`:

```
git tag -a vX.Y -m "FireDog vX.Y" <head-sha>
```

The tree must be clean at this point (a dirty tree builds `vX.Y-dirty`).

Before building, make sure no other build is using the folder
(`Get-Process ninja, cl, link -ErrorAction SilentlyContinue`) - a concurrent
build fails with "ninja: error: failed recompaction: Permission denied". If one
is running, it is probably the user's: wait for it rather than start a second.

Then build in the BACKGROUND (a release build from a fresh folder takes
minutes; never block a foreground call on it), logging to the scratchpad:

```
$vc = "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"
cmd /c "`"$vc`" >nul 2>&1 && cmake --build out/build/x64-release-deploy" *> <scratchpad>\build-vX.Y.log
```

(PowerShell tool, `run_in_background: true`; you are notified when it ends.)
On failure, report the first error lines from the log and stop - do not pack.

Verify, whatever ran the build:

```
Get-Item out\build\x64-release-deploy\FireDog.exe, out\build\x64-release-deploy\FireDogRig.exe | Select-Object Name, Length, LastWriteTime
Get-Content out\build\x64-release-deploy\generated\firedog_gitinfo.h
```

Both exe timestamps must be newer than HEAD's commit time
(`git log -1 --format=%cI`), and `FD_GIT_DESCRIBE` must be exactly `vX.Y` (no
`-dirty`, no `-N-g…`). Twice the "built" binary turned out to be
`out\build\x64-debug\FireDog.exe` (17 MB, debug runtime) while the deploy exe
was weeks old - never take a timestamp on trust. A release from a stale,
dirty or debug binary is worse than a delay.

Read the build log's tail for warnings and keep the count for the report.

If a commit lands AFTER the tag but before publishing (e.g. a packaging or
docs fix that does not change the exe), leave the tag on the built commit -
the tag names the source of the binary - and push master as well.

## 3. Pack

```
tools\release\package-release.ps1 -Version vX.Y -OutDir <scratchpad>
```

It refuses to run without a deploy exe or without a machine root to seed
from. From v0.13 the zip carries: the two exes, every runtime `.dll`/`.cti`
the preset staged beside them (pylon, PI, Ezi-IO), `assets/`, the REPO's
`config/scripts/` (code - it must match the exe), and a SEED of this
machine's root (`-SeedRoot`, default `D:\ProgramData\FireDog`): its whole
`config/` (hardware.json, transforms, every `*.local.json` incl. needle_cal
and device_reg) and `products/` (the product documents AND `employees.json`
with its plaintext passwords), minus `*.bak` / `*.zip`. The repo's
`config/*.json` defaults are no longer shipped. The seed sits beside the exe
where the first-start migration (core/Paths) copies it into an EMPTY machine
root; an existing root is never touched. The user chose this on 2026-10-08,
knowing the seed is one machine's - say so in the notes' Build section. No
libs, no CMake state, no logs, no data/. Expect about 15 MB. The 100 MB
`FireDog.zip` / `x64-release-deploy.zip` lying in `out/build` are raw
directory dumps and never what shipped. List the zip's top level in the
report.

## 3b. Velopack packages (after the notes in step 4 exist)

```
tools\release\pack-velopack.ps1 -Version vX.Y -OutDir <scratchpad> -NotesFile <notes.md>
```

It unzips the step-3 zip and packs THAT folder, so an installed station runs
exactly the zip's files. It fetches the previous release's full package first
for a delta (the first Velopack release has none - fine), and writes
`Install-FireDog.cmd` (Setup with `--installto C:\FireDog`, the station folder).
vpk runs as `dotnet tool exec vpk@1.2.161` - the .NET 10 SDK is installed here.
**That version must equal `velopack_libc` in CMakeLists.txt**; bump both
together. Velopack only offers a STRICTLY newer SemVer: `vX.Y` packs as
`X.Y.0`, so every release must be a new version (it always is). Unsigned:
SmartScreen warns on a station's first Setup run.

## 4. Notes

`git log --format='%ad %s' --date=short <last-tag>..HEAD` is the source. If a
tag exists that was never released (v0.11 was), the range is from the last
*released* tag and the notes say so. Write them to a scratchpad `.md` in the
style of the v0.1 and v0.2 releases (`gh release view v0.2 --json body`):

- one line up top: N commits, date range;
- `##` sections by area (Programs and the Run deck, Recipes, Motion,
  Registration/gripped objects/placement, Instruments and access, Build), each a
  bulleted list with the feature bolded and one sentence on what it means for
  the operator - not the commit subjects verbatim;
- the Build section ends with the deployment caveat: the package carries the
  pylon runtime but camera drivers, `ACSCL_x64.dll`, `VISA64.dll` and the VC++
  2015-2022 x64 redistributable must be present on the target (the Setup
  installs the VC++ runtime itself; the hand-unzipped zip does not).

## 5. Tag, publish, verify

The tag was made locally in step 2, on the commit that was built. Push master
first, then the tag:

```
git push origin master && git push origin vX.Y
gh release create vX.Y <zip> --title "FireDog vX.Y" --notes-file <notes.md> --latest
tools\release\pack-velopack.ps1 -Version vX.Y -OutDir <scratchpad> -Upload
gh release view vX.Y --json tagName,name,isDraft,targetCommitish,assets,url
gh release list --limit 3
```

`isLatest` is not a field `gh release view` knows; `gh release list` is what
shows the Latest marker. The assets must include `releases.win.json` and a
`FireDog-X.Y.0-full.nupkg` - without them no installed station sees the
release. If the upload failed, re-run only the `-Upload` line.

The notes' Build section says how a NEW station installs: download
`FireDog-win-Setup.exe` and `Install-FireDog.cmd` into one folder and run the
`.cmd` (installs to `C:\FireDog`); an installed station updates from Help >
Version / Update.

## 6. Report

Lead with the release URL. Then: tag and commit, asset name and size, the build
warning count (name the files), and whether you ran the build yourself. Keep
the packaging script and this skill in mind for follow-ups the user may want.

## Pitfalls seen so far

- `.gitignore` carries the stock Visual Studio `[Rr]elease/` rule, which
  swallows any directory named `release` - `tools/release/` and this skill's
  own folder are un-ignored explicitly. A new `release/` folder needs the same.
- `cmake` is not on PATH in Git Bash; every cmake call goes through the VS 18
  vcvars in PowerShell (`cmd /c "call vcvars64.bat && cmake ..."`).
- `gh` (GitHub CLI) was installed with winget on 2026-10-08 at
  `C:\Program Files\GitHub CLI\gh.exe`. A session started before an install
  does not have it on PATH - call it by that full path. If `gh auth status`
  says not logged in, the user runs
  `! & "C:\Program Files\GitHub CLI\gh.exe" auth login --web` (interactive,
  so it cannot be done for them).
- Do not create the release before the tag is pushed, and do not tag before
  master is pushed: `gh release create` on an unknown tag makes its own tag on
  the default branch head, which may not be the commit you packaged.
