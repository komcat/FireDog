# package-release.ps1 : the release zip, from the x64-release-deploy build.
#
#   1. cmake --preset x64-release-deploy       (from the VS developer prompt)
#   2. cmake --build out/build/x64-release-deploy
#   3. tools\release\package-release.ps1 -Version v0.3
#   4. git tag -a v0.3 -m "FireDog v0.3" && git push origin v0.3
#   5. gh release create v0.3 FireDog-v0.3-x64-release-deploy.zip `
#          --title "FireDog v0.3" --notes-file <notes.md>
#
# What goes in, and why exactly that:
#
#   * the two exes and every runtime DLL / .cti beside them (the deploy preset
#     stages the pylon, PI and Ezi-IO runtimes there so a target with none of
#     them installed still STARTS - see the v0.1 release notes for what it does
#     not get you), and assets/.
#
#   * config/scripts/ from the REPOSITORY. The standard scripts are code: they
#     change with the commands they call, so they must be the ones this exe was
#     built with - a machine's older copy would silently skip the newest step.
#
#   * a SEED of a running machine's state, taken from its machine root
#     (-SeedRoot, default D:\ProgramData\FireDog): the rest of its config/
#     (hardware.json, motion_transforms.json, every *.local.json and journal,
#     vision/) and its products/ (the product documents and employees.json).
#     It is laid out beside the exe where the FIRST-START MIGRATION (core/Paths)
#     looks, so a target whose machine root is empty copies it in on its first
#     run; a target that already has a root is never touched (the migration
#     only runs against an empty root, and the seeding of hardware.json /
#     motion_transforms.json never overwrites). The repo's config/*.json
#     defaults are NOT shipped: the seed's *.local.json files take their place.
#
#     The seed is one machine's: its needle calibration, device registration
#     and taught positions describe THAT machine, and employees.json carries
#     the roster's plaintext passwords. Shipping them is deliberate (a
#     reinstall, or a sister machine started from a known state) - re-measure
#     the needle and the registration on any other machine before moving.
#
# Never shipped: the root's data/, logs/, imgui.ini, migrated_from.txt, *.bak
# and *.zip leftovers; no libs, no CMake state, no FireDog.ini.
param(
    [Parameter(Mandatory = $true)][string]$Version,   # e.g. v0.3
    [string]$BuildDir = "out\build\x64-release-deploy",
    [string]$SeedRoot = "D:\ProgramData\FireDog",
    [string]$OutDir = "."
)
$ErrorActionPreference = "Stop"
if (-not (Test-Path (Join-Path $BuildDir "FireDog.exe"))) {
    throw "no FireDog.exe in $BuildDir - build the x64-release-deploy preset first"
}
if (-not (Test-Path (Join-Path $SeedRoot "config\hardware.json"))) {
    throw "no config\hardware.json under $SeedRoot - -SeedRoot must be a machine root"
}
$name  = "FireDog-$Version-x64-release-deploy"
$stage = Join-Path $OutDir $name
$zip   = Join-Path $OutDir "$name.zip"
if (Test-Path $stage) { Remove-Item -Recurse -Force $stage }
if (Test-Path $zip)   { Remove-Item -Force $zip }
New-Item -ItemType Directory -Force $stage | Out-Null

Get-ChildItem $BuildDir -File | Where-Object { $_.Extension -in ".exe", ".dll", ".cti" } |
    ForEach-Object { Copy-Item $_.FullName $stage }

# Copy a tree's files to $to, keeping relative paths; $skip filters on the
# relative path.
function Copy-Tree([string]$from, [string]$to, [scriptblock]$skip) {
    $from = (Resolve-Path $from).Path
    Get-ChildItem $from -Recurse -File | ForEach-Object {
        $rel = $_.FullName.Substring($from.Length + 1)
        if (& $skip $rel) { return }
        $dest = Join-Path $to $rel
        New-Item -ItemType Directory -Force (Split-Path $dest) | Out-Null
        Copy-Item $_.FullName $dest
    }
}
$leftover = { param($rel) $rel -like "*.bak" -or $rel -like "*.zip" }

# Scripts: the repository's, matching this exe.
$cfg = Join-Path $stage "config"
Copy-Tree (Join-Path $PSScriptRoot "..\..\config\scripts") (Join-Path $cfg "scripts") { $false }

# The seed: the machine's config/ minus its scripts, and its products/.
Copy-Tree (Join-Path $SeedRoot "config") $cfg {
    param($rel) $rel -like "scripts\*" -or (& $leftover $rel)
}
if (Test-Path (Join-Path $SeedRoot "products")) {
    Copy-Tree (Join-Path $SeedRoot "products") (Join-Path $stage "products") $leftover
}

if (Test-Path (Join-Path $BuildDir "assets")) {
    Copy-Item (Join-Path $BuildDir "assets") (Join-Path $stage "assets") -Recurse
}

Compress-Archive -Path $stage -DestinationPath $zip -CompressionLevel Optimal
Remove-Item -Recurse -Force $stage
$size = [math]::Round((Get-Item $zip).Length / 1MB, 1)
Write-Output "$zip  ($size MB)  seed from $SeedRoot"
