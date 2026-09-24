---
name: release
description: Cut a FireDog release - configure the x64-release-deploy preset, get a fresh build, pack the deploy zip with tools/release/package-release.ps1, write release notes from the commits since the last tag, tag vX.Y, and publish a GitHub release with the zip attached. Use this whenever the user asks to release, publish, deploy, package, ship, tag a version, "make a build for the machine", "put it on the release page", or attach an exe/zip to GitHub - even if they only say "v0.3" or "release it" with no other words.
---

# Release

The deliverable is a GitHub release at `github.com/komcat/FireDog/releases` with
one asset, `FireDog-vX.Y-x64-release-deploy.zip`, and notes by area. Everything
below exists because a step was skipped or done wrong once; the order matters.

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

## 2. The build is the user's - then VERIFY it landed

The project rule is that the user compiles. Hand them the one command, which
runs the build inside the VS 18 developer environment:

```
! cmd /c "call \"C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat\" && cmake --build out/build/x64-release-deploy"
```

When they say it is built, do not take their word for it - check:

```
Get-Item out\build\x64-release-deploy\FireDog.exe, out\build\x64-release-deploy\FireDogRig.exe | Select-Object Name, Length, LastWriteTime
```

Both timestamps must be newer than HEAD's commit time (`git log -1 --format=%cI`).
Twice in a row the "built" binary turned out to be `out\build\x64-debug\FireDog.exe`
(17 MB, debug runtime) because the IDE's active configuration was Debug; the
deploy exe was still weeks old. Say exactly that - which exe is fresh, which is
stale - and give the command again. If the user has explicitly delegated
("go ahead") and the build still has not landed, run the command yourself in
the background with a log in the scratchpad, and say plainly in the report
that you did. A release from a stale or debug binary is worse than a delay.

Read the build log's tail for warnings and keep the count for the report.

## 3. Pack

```
tools\release\package-release.ps1 -Version vX.Y -OutDir <scratchpad>
```

It refuses to run without a deploy exe, and it produces exactly the shape the
v0.1 and v0.2 assets had: the two exes, every runtime `.dll`/`.cti` the preset
staged beside them (pylon, PI, Ezi-IO), `config/` minus `*.local.json` (those
are THIS machine's taught positions and UI layout), and `assets/`. No libs,
no CMake state, no logs. Expect about 15 MB. The 100 MB `FireDog.zip` /
`x64-release-deploy.zip` lying in `out/build` are raw directory dumps and
never what shipped. List the zip's top level in the report.

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
  2015-2022 x64 redistributable must be present on the target.

## 5. Tag, publish, verify

```
git tag -a vX.Y -m "FireDog vX.Y" <head-sha> && git push origin vX.Y
gh release create vX.Y <zip> --title "FireDog vX.Y" --notes-file <notes.md> --latest
gh release view vX.Y --json tagName,name,isDraft,targetCommitish,assets,url
gh release list --limit 3
```

`isLatest` is not a field `gh release view` knows; `gh release list` is what
shows the Latest marker.

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
- Do not create the release before the tag is pushed, and do not tag before
  master is pushed: `gh release create` on an unknown tag makes its own tag on
  the default branch head, which may not be the commit you packaged.
