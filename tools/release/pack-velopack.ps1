# pack-velopack.ps1 : the Velopack update packages for a release, built FROM the
# release zip package-release.ps1 made, so the installed copy and the zip are the
# same files. docs/software-update-design.md.
#
#   tools\release\pack-velopack.ps1 -Version v0.14 -Zip <zip> -NotesFile <notes.md>
#       (after package-release.ps1, before gh release create)
#   tools\release\pack-velopack.ps1 -Version v0.14 -Upload
#       (after gh release create: adds the packages to that release)
#
# What it produces, in <OutDir>\velopack\:
#
#   FireDog-win-Setup.exe       first install on a new station
#   Install-FireDog.cmd         runs that Setup with --installto C:\FireDog - the
#                               station folder. Setup alone installs to
#                               %LOCALAPPDATA%\FireDog; vpk has no option to bake a
#                               default path in, so the path travels in this file.
#   FireDog-win-Portable.zip    the same install unzipped by hand (also updatable)
#   FireDog-<ver>-full.nupkg    what an installed station downloads ...
#   FireDog-<ver>-delta.nupkg   ... or only the difference from the previous one
#   releases.win.json           the feed AppUpdater reads
#
# vpk runs pinned through `dotnet tool exec` (needs the .NET 10 SDK, no install).
# Its version MUST equal the velopack_libc version in CMakeLists.txt.
#
# Velopack versions are SemVer and only a strictly NEWER one is offered: tag
# v0.14 packs as 0.14.0, v0.14.1 as itself.
param(
    [Parameter(Mandatory = $true)][string]$Version,   # e.g. v0.14
    [string]$Zip,                                     # default <OutDir>\FireDog-<Version>-x64-release-deploy.zip
    [string]$NotesFile,
    [string]$OutDir = ".",
    [switch]$Upload
)
$ErrorActionPreference = "Stop"

$VpkVersion = "1.2.161"
$RepoUrl    = "https://github.com/komcat/FireDog"
$InstallDir = "C:\FireDog"
$repo       = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$work       = Join-Path $OutDir "velopack"
$rel        = Join-Path $work "releases"

$gh = (Get-Command gh -ErrorAction SilentlyContinue).Source
if (-not $gh) { $gh = "C:\Program Files\GitHub CLI\gh.exe" }
$token = (& $gh auth token).Trim()
if (-not $token) { throw "gh is not logged in - run: gh auth login" }

function Invoke-Vpk([string[]]$VpkArgs) {
    & dotnet tool exec "vpk@$VpkVersion" --yes "--" @VpkArgs | Out-Host
    return $LASTEXITCODE
}

if ($Upload) {
    if (-not (Test-Path (Join-Path $rel "releases.win.json"))) {
        throw "nothing packed in $rel - run without -Upload first"
    }
    $code = Invoke-Vpk @("upload", "github", "--outputDir", $rel, "--repoUrl", $RepoUrl,
                         "--token", $token, "--merge", "--tag", $Version,
                         "--releaseName", "FireDog $Version", "--publish")
    if ($code -ne 0) { throw "vpk upload failed - the release exists without its update packages; re-run with -Upload" }
    & $gh release upload $Version (Join-Path $rel "Install-FireDog.cmd") --clobber
    if ($LASTEXITCODE -ne 0) { throw "gh release upload Install-FireDog.cmd failed" }
    Write-Output "update packages added to $Version"
    return
}

# ---- pack ----
$ver = $Version.TrimStart("v")
if ($ver -notmatch '^\d+\.\d+\.\d+') { $ver = "$ver.0" }
if (-not $Zip) { $Zip = Join-Path $OutDir "FireDog-$Version-x64-release-deploy.zip" }
if (-not (Test-Path $Zip)) { throw "no release zip at $Zip - run package-release.ps1 first" }
if (-not $NotesFile -or -not (Test-Path $NotesFile)) { throw "-NotesFile must name the release notes" }

if (Test-Path $work) { Remove-Item -Recurse -Force $work }
New-Item -ItemType Directory -Force $rel | Out-Null
$unz = Join-Path $work "unzipped"
Expand-Archive -Path $Zip -DestinationPath $unz
$packDir = (Get-ChildItem $unz -Directory | Select-Object -First 1).FullName
if (-not (Test-Path (Join-Path $packDir "FireDog.exe"))) { throw "no FireDog.exe at the top of $Zip" }

# The previous release's full package lets vpk make a delta. The first Velopack
# release has none and ships the full package only.
Write-Output "==> previous update package (for the delta)"
if ((Invoke-Vpk @("download", "github", "--repoUrl", $RepoUrl, "--token", $token,
                  "--outputDir", $rel)) -ne 0) {
    Write-Output "   (none - this release carries the full package only)"
}

# vcredist143-x64: Setup installs the VC++ 2015-2022 runtime when the station
# lacks it, so a new PC needs nothing but the Setup.
Write-Output "==> vpk pack FireDog $ver"
$code = Invoke-Vpk @("pack", "--packId", "FireDog", "--packVersion", $ver,
                     "--packDir", $packDir, "--mainExe", "FireDog.exe",
                     "--packTitle", "FireDog", "--packAuthors", "Fabrinet",
                     "--runtime", "win-x64", "--framework", "vcredist143-x64",
                     "--icon", (Join-Path $repo "resources\FireDog.ico"),
                     "--releaseNotes", $NotesFile, "--outputDir", $rel)
if ($code -ne 0) { throw "vpk pack failed" }

@"
@echo off
rem Installs FireDog into $InstallDir, the station folder. Keep this file beside
rem FireDog-win-Setup.exe. Later releases arrive through Help > Version / Update.
"%~dp0FireDog-win-Setup.exe" --installto $InstallDir
"@ | Set-Content -Encoding ascii (Join-Path $rel "Install-FireDog.cmd")

Remove-Item -Recurse -Force $unz
Get-ChildItem $rel -File | ForEach-Object {
    "{0}  ({1} MB)" -f $_.Name, [math]::Round($_.Length / 1MB, 1)
}
