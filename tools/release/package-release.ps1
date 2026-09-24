# package-release.ps1 : the release zip, from the x64-release-deploy build.
#
#   1. cmake --preset x64-release-deploy       (from the VS developer prompt)
#   2. cmake --build out/build/x64-release-deploy
#   3. tools\release\package-release.ps1 -Version v0.3
#   4. git tag -a v0.3 -m "FireDog v0.3" && git push origin v0.3
#   5. gh release create v0.3 FireDog-v0.3-x64-release-deploy.zip `
#          --title "FireDog v0.3" --notes-file <notes.md>
#
# What goes in, and why exactly that: the two exes and every runtime DLL / .cti
# beside them (the deploy preset stages the pylon, PI and Ezi-IO runtimes there
# so a target with none of them installed still STARTS - see the v0.1 release
# notes for what it does not get you), config/ minus the runtime-writable
# *.local.json files (those describe THIS machine: its taught positions, its
# UI layout), and assets/. No libs, no CMake state, no logs, no imgui.ini.
# The 100 MB zip of the whole build directory that used to sit in out/build
# was never what shipped.
param(
    [Parameter(Mandatory = $true)][string]$Version,   # e.g. v0.3
    [string]$BuildDir = "out\build\x64-release-deploy",
    [string]$OutDir = "."
)
$ErrorActionPreference = "Stop"
if (-not (Test-Path (Join-Path $BuildDir "FireDog.exe"))) {
    throw "no FireDog.exe in $BuildDir - build the x64-release-deploy preset first"
}
$name  = "FireDog-$Version-x64-release-deploy"
$stage = Join-Path $OutDir $name
$zip   = Join-Path $OutDir "$name.zip"
if (Test-Path $stage) { Remove-Item -Recurse -Force $stage }
if (Test-Path $zip)   { Remove-Item -Force $zip }
New-Item -ItemType Directory -Force $stage | Out-Null

Get-ChildItem $BuildDir -File | Where-Object { $_.Extension -in ".exe", ".dll", ".cti" } |
    ForEach-Object { Copy-Item $_.FullName $stage }

$cfgSrc = (Resolve-Path (Join-Path $BuildDir "config")).Path
$cfg    = Join-Path $stage "config"
New-Item -ItemType Directory -Force $cfg | Out-Null
Get-ChildItem $cfgSrc -Recurse -File |
    Where-Object { $_.Name -notlike "*.local.json" } |
    ForEach-Object {
        $rel  = $_.FullName.Substring($cfgSrc.Length + 1)
        $dest = Join-Path $cfg $rel
        New-Item -ItemType Directory -Force (Split-Path $dest) | Out-Null
        Copy-Item $_.FullName $dest
    }
if (Test-Path (Join-Path $BuildDir "assets")) {
    Copy-Item (Join-Path $BuildDir "assets") (Join-Path $stage "assets") -Recurse
}

Compress-Archive -Path $stage -DestinationPath $zip -CompressionLevel Optimal
Remove-Item -Recurse -Force $stage
$size = [math]::Round((Get-Item $zip).Length / 1MB, 1)
Write-Output "$zip  ($size MB)"
