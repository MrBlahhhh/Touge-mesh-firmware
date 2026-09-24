# Stage a Touge build for the web flasher: copy the built images out of the
# Meshtastic checkout, check them, and add the build to releases.json.
#
# Building stays manual, as it is for flash-all.ps1. Build first, in the
# checkout, with a plain `pio run -e <env>`: the plain run also makes the
# mtjson target, which is what builds the littlefs image and the .mt.json. A
# `pio run -t upload` skips both, which is why the V3 build dir has neither.
#
# What it writes into -Site (the Pages site, see README.md):
#
#   index.html, flasher.js, style.css      the page, copied from here
#   releases.json                          every published build, newest first
#   firmware/<build>/<env>/...             the images for that build
#
# GPL: a published binary needs its exact source published too. So unless
# -Bench is given, this refuses when the source repo (-Source, a checkout of
# Touge-mesh-firmware) does not hold the overlay and core patches that are in
# the Meshtastic checkout right now, or has uncommitted changes. The commit it
# records is the one the page links as the source.
#
#   .\publish-release.ps1                                  check, stage both boards
#   .\publish-release.ps1 -Notes "Fixes the V3 boot loop"  one line shown on the page
#   .\publish-release.ps1 -Bench -Site C:\tmp\flasher      stage for a bench test, source unchecked
#   .\publish-release.ps1 -Boards heltec-v4                only this board

param(
    [string]$Checkout = "C:\Projects\meshtastic-firmware",
    [string]$Source = "C:\Projects\touge-mesh-firmware",
    [string]$Site = "C:\Projects\touge-mesh-firmware-pages",
    [string[]]$Boards = @("heltec-v3", "heltec-v4"),
    [string]$Notes = "",
    [switch]$Bench
)

$ErrorActionPreference = "Stop"
$SourceRepoUrl = "https://github.com/MrBlahhhh/Touge-mesh-firmware"

# Run git and return its stdout. Continue, not Stop: Windows PowerShell turns
# any stderr line from a native program into a terminating error.
function Run-Git([string]$dir) {
    $ErrorActionPreference = "Continue"
    $out = & git.exe -C $dir @args 2>$null
    return @{ Out = $out; Ok = ($LASTEXITCODE -eq 0) }
}

function Hex([uint32]$n) { return "0x" + $n.ToString("x") }

function Hash([string]$path, [string]$algorithm) {
    return (Get-FileHash -Algorithm $algorithm -LiteralPath $path).Hash.ToLower()
}

function Md5-Bytes([byte[]]$bytes, [int]$offset = 0, [int]$count = -1) {
    if ($count -lt 0) { $count = $bytes.Length - $offset }
    $md5 = [System.Security.Cryptography.MD5]::Create()
    return (($md5.ComputeHash($bytes, $offset, $count) | ForEach-Object { $_.ToString("x2") }) -join "")
}

# Hash ignoring CR, so a checkout with autocrlf compares equal to one without.
function Text-Hash([string]$path) {
    $text = [System.IO.File]::ReadAllText($path).Replace("`r", "")
    return Md5-Bytes ([System.Text.Encoding]::UTF8.GetBytes($text))
}

# Byte-by-byte loops are minutes in PowerShell on a 2 MB image; md5 of the
# slice is milliseconds.
function Same-Bytes([byte[]]$a, [int]$offset, [byte[]]$b) {
    if ($a.Length -lt $offset + $b.Length) { return $false }
    return (Md5-Bytes $a $offset $b.Length) -eq (Md5-Bytes $b)
}

# partitions.bin is 32-byte records: AA 50, type, subtype, offset, size,
# 16-byte label, flags. It ends at an EB EB md5 record or at erased flash.
function Read-PartitionTable([string]$path) {
    $bytes = [System.IO.File]::ReadAllBytes($path)
    $parts = [ordered]@{}
    for ($i = 0; $i + 32 -le $bytes.Length; $i += 32) {
        if ($bytes[$i] -ne 0xAA -or $bytes[$i + 1] -ne 0x50) { break }
        $label = [System.Text.Encoding]::ASCII.GetString($bytes, $i + 12, 16).TrimEnd([char]0)
        $parts[$label] = [pscustomobject]@{
            Offset = [BitConverter]::ToUInt32($bytes, $i + 4)
            Size = [BitConverter]::ToUInt32($bytes, $i + 8)
        }
    }
    return $parts
}

# ---------------------------------------------------------------- what was built

$touge = Get-Content (Join-Path $Checkout "src\modules\esp32\TougeFastModule.cpp") -Raw
$buildMatch = [regex]::Match($touge, 'TOUGE_BUILD\s*=\s*(\d+)')
if (-not $buildMatch.Success) { throw "no TOUGE_BUILD in the checkout's TougeFastModule.cpp; is the overlay applied?" }
$build = [int]$buildMatch.Groups[1].Value

$head = Run-Git $Checkout rev-parse HEAD
if (-not $head.Ok) { throw "$Checkout is not a git checkout" }
$meshtasticCommit = "$($head.Out)".Trim()

Write-Host "Touge build $build on Meshtastic $($meshtasticCommit.Substring(0, 7))"

# ---------------------------------------------------------------- does the source match

# Everything the overlay put into the checkout, keyed by path under src\.
$overlayInCheckout = @(Get-ChildItem (Join-Path $Checkout "src\touge") -File -Recurse) +
    @(Get-ChildItem (Join-Path $Checkout "src\modules\esp32") -File -Filter "TougeFast*")

function Source-Problems {
    $problems = @()
    if (-not (Test-Path (Join-Path $Source "overlay\src"))) {
        return @("$Source has no overlay\src; is it a Touge-mesh-firmware checkout?")
    }
    $status = Run-Git $Source status --porcelain --untracked-files=no -- .
    if ($status.Out) { $problems += "$Source has uncommitted changes" }

    $srcRoot = Join-Path $Checkout "src"
    $overlayRoot = Join-Path $Source "overlay\src"
    foreach ($file in $overlayInCheckout) {
        $rel = $file.FullName.Substring($srcRoot.Length + 1)
        $theirs = Join-Path $overlayRoot $rel
        if (-not (Test-Path $theirs)) { $problems += "src\$rel is in the build but not in the source repo"; continue }
        if ((Text-Hash $file.FullName) -ne (Text-Hash $theirs)) { $problems += "src\$rel differs from the source repo" }
    }
    foreach ($file in Get-ChildItem $overlayRoot -File -Recurse) {
        $rel = $file.FullName.Substring($overlayRoot.Length + 1)
        if (-not (Test-Path (Join-Path $srcRoot $rel))) { $problems += "src\$rel is in the source repo but not in the build" }
    }

    # Core files the build changed beyond the overlay must each come from a
    # patch the source repo carries, and that patch must be applied.
    $covered = @("src/modules/Modules.cpp")   # apply-overlay.sh's own two edits
    foreach ($patch in @(Get-ChildItem (Join-Path $Source "core-patches") -Filter *.patch -ErrorAction SilentlyContinue)) {
        $covered += Select-String -LiteralPath $patch.FullName -Pattern '^\+\+\+ b/(.+)$' | ForEach-Object { $_.Matches[0].Groups[1].Value.Trim() }
        $applied = Run-Git $Checkout apply --check --reverse $patch.FullName
        if (-not $applied.Ok) { $problems += "core patch $($patch.Name) is not applied in the checkout" }
    }
    $changed = Run-Git $Checkout diff --name-only HEAD
    foreach ($path in @($changed.Out)) {
        if (-not $path -or $path -like ".vscode/*") { continue }
        if ($covered -notcontains $path) { $problems += "$path is changed in the checkout and no core patch in the source repo covers it" }
    }
    return $problems
}

$sourceCommit = $null
$problems = @(Source-Problems)
if ($problems) {
    Write-Host "The source repo does not match this build:"
    $problems | ForEach-Object { Write-Host "  $_" }
    if (-not $Bench) {
        throw "not publishing a binary whose source is not published (GPL). Bring $Source up to date and push it, or use -Bench for a local test."
    }
    Write-Host "-Bench: staging anyway, marked as a bench build with no source link."
} elseif (-not $Bench) {
    $sourceCommit = "$((Run-Git $Source rev-parse HEAD).Out)".Trim()
    $pushed = Run-Git $Source branch -r --contains $sourceCommit
    if (-not $pushed.Out) { Write-Host "WARNING: $($sourceCommit.Substring(0, 7)) is not on any remote branch yet. Push it before the page goes live." }
}

# ---------------------------------------------------------------- stage each board

$newestSource = ($overlayInCheckout | Measure-Object -Property LastWriteTime -Maximum).Maximum
$boardEntries = [ordered]@{}
$builtDates = @()

foreach ($envName in $Boards) {
    Write-Host "== $envName"
    $dir = Join-Path $Checkout ".pio\build\$envName"
    $app = Get-ChildItem $dir -Filter "firmware-$envName-*.bin" | Where-Object { $_.Name -notlike "*.factory.bin" } | Select-Object -First 1
    $factory = Get-ChildItem $dir -Filter "firmware-$envName-*.factory.bin" | Select-Object -First 1
    $tablePath = Join-Path $dir "partitions.bin"
    if (-not $app -or -not $factory -or -not (Test-Path $tablePath)) { throw "$envName is not built in $dir" }
    $littlefs = Get-ChildItem $dir -Filter "littlefs-$envName-*.bin" -ErrorAction SilentlyContinue | Select-Object -First 1
    $mtJson = Get-ChildItem $dir -Filter "firmware-$envName-*.mt.json" -ErrorAction SilentlyContinue | Select-Object -First 1

    if ($app.LastWriteTime -lt $newestSource) {
        throw "$($app.Name) is older than the overlay sources in the checkout; rebuild $envName first"
    }
    $builtDates += $app.LastWriteTime
    $meshtasticVersion = [regex]::Match($app.Name, "firmware-$envName-(.+)\.bin").Groups[1].Value

    $parts = Read-PartitionTable $tablePath
    foreach ($name in @("nvs", "otadata", "app0", "spiffs")) {
        if (-not $parts.Contains($name)) { throw "$envName partition table has no $name" }
    }
    # The page writes an update at these two addresses and never the rest; a
    # table that moved them is a different layout and needs a fresh install.
    if ($parts["app0"].Offset -ne 0x10000 -or $parts["otadata"].Offset -ne 0xe000) {
        throw "$envName has app0 at $(Hex $parts['app0'].Offset) and otadata at $(Hex $parts['otadata'].Offset), expected 0x10000 and 0xe000"
    }

    # The factory image is bootloader + table + otadata + app, laid out from
    # 0x0. Check it really contains this table and this app before shipping it.
    $factoryBytes = [System.IO.File]::ReadAllBytes($factory.FullName)
    $tableBytes = [System.IO.File]::ReadAllBytes($tablePath)
    $appBytes = [System.IO.File]::ReadAllBytes($app.FullName)
    if (-not (Same-Bytes $factoryBytes 0x8000 $tableBytes)) { throw "$($factory.Name) does not hold partitions.bin at 0x8000" }
    if (-not (Same-Bytes $factoryBytes 0x10000 $appBytes)) { throw "$($factory.Name) does not hold $($app.Name) at 0x10000" }

    # Why Update writes otadata as well as the app.
    #
    # The bootloader runs app0 or app1, whichever otadata names. This build
    # can end up on app1: MeshtasticOTA::trySwitchToOTA() (AdminModule, on an
    # OTA request from a Meshtastic client) calls esp_ota_set_boot_partition()
    # on the ota_1 slot when that slot holds Meshtastic's OTA loader, and
    # flasher.meshtastic.org writes that loader into app1 on its own updates.
    # A radio flashed there before, or left mid-OTA, boots app1, and writing
    # app0 alone would change nothing it runs.
    #
    # So Update also writes the initial otadata (ota_seq 1 = app0, what
    # boot_app0.bin holds and what `pio run -t upload` writes to 0xe000), cut
    # from the factory image so it is exactly what this build ships with. It
    # is written after the app: if the write stops in between, otadata still
    # names whatever the radio booted before. NVS at 0x9000 (the Bluetooth
    # bond) and the littlefs partition (the settings) are never written.
    $otadataBytes = New-Object byte[] 0x2000
    [Array]::Copy($factoryBytes, 0xe000, $otadataBytes, 0, 0x2000)
    if ([BitConverter]::ToUInt32($otadataBytes, 0) -ne 1) {
        throw "$($factory.Name) otadata at 0xe000 does not start with ota_seq 1; not the boot_app0 layout this script expects"
    }

    if ($mtJson) {
        $mt = Get-Content $mtJson.FullName -Raw | ConvertFrom-Json
        $mtApp = $mt.files | Where-Object { $_.name -eq $app.Name }
        if ($mtApp -and $mtApp.md5 -ne (Hash $app.FullName MD5)) { throw "$($mtJson.Name) md5 for $($app.Name) does not match the file; stale manifest" }
        foreach ($p in $mt.part) {
            if ($parts.Contains($p.name) -and [Convert]::ToUInt32($p.offset.Replace("0x", ""), 16) -ne $parts[$p.name].Offset) {
                throw "$($mtJson.Name) puts $($p.name) at $($p.offset), partitions.bin at $(Hex $parts[$p.name].Offset)"
            }
        }
    }

    $outDir = Join-Path $Site "firmware\$build\$envName"
    New-Item -ItemType Directory -Force $outDir | Out-Null
    Copy-Item $app.FullName, $factory.FullName -Destination $outDir -Force
    $otadataName = "otadata-boot-app0.bin"
    [System.IO.File]::WriteAllBytes((Join-Path $outDir $otadataName), $otadataBytes)

    function File-Entry([string]$name, [uint32]$offset, [string]$what) {
        $path = Join-Path $outDir $name
        return [ordered]@{
            what = $what
            path = "firmware/$build/$envName/$name"
            offset = Hex $offset
            size = (Get-Item $path).Length
            md5 = Hash $path MD5
            sha256 = Hash $path SHA256
        }
    }

    $update = @(
        (File-Entry $app.Name 0x10000 "app"),
        (File-Entry $otadataName 0xe000 "otadata")
    )
    $fresh = @(File-Entry $factory.Name 0x0 "factory")
    if ($littlefs) {
        if ($littlefs.Length -ne $parts["spiffs"].Size) { throw "$($littlefs.Name) is $($littlefs.Length) bytes, the spiffs partition is $($parts['spiffs'].Size)" }
        Copy-Item $littlefs.FullName -Destination $outDir -Force
        $fresh += File-Entry $littlefs.Name $parts["spiffs"].Offset "littlefs"
    } else {
        # No image: the fresh install erases the chip, so the partition is
        # blank, and the firmware's FSCom.begin(true) formats it on first
        # boot. data\ only holds static\.gitkeep, so a built image would be an
        # empty file system anyway.
        Write-Host "  no littlefs image: fresh install writes the factory image only, the radio formats its file system on first boot"
    }

    $partMap = [ordered]@{}
    foreach ($name in $parts.Keys) { $partMap[$name] = Hex $parts[$name].Offset }

    $boardEntries[$envName] = [ordered]@{
        partitionTable = [ordered]@{ offset = "0x8000"; size = $tableBytes.Length; md5 = Md5-Bytes $tableBytes }
        partitions = $partMap
        update = $update
        fresh = $fresh
    }
    Write-Host "  update: $(($update | ForEach-Object { "$($_.offset) $($_.what)" }) -join ', ')"
    Write-Host "  fresh:  erase, $(($fresh | ForEach-Object { "$($_.offset) $($_.what)" }) -join ', ')"
}

# ---------------------------------------------------------------- releases.json

$release = [ordered]@{
    build = $build
    date = (($builtDates | Measure-Object -Maximum).Maximum).ToString("yyyy-MM-dd")
    bench = [bool]$Bench
    notes = $Notes
    sourceCommit = $sourceCommit
    meshtasticVersion = $meshtasticVersion
    meshtasticCommit = $meshtasticCommit
    boards = $boardEntries
}

$manifestPath = Join-Path $Site "releases.json"
$others = @()
if (Test-Path $manifestPath) {
    $others = @((Get-Content $manifestPath -Raw | ConvertFrom-Json).releases | Where-Object { $_.build -ne $build })
}
$releases = @(@($release) + $others | Sort-Object -Property { [int]$_.build } -Descending)
$manifest = [ordered]@{ schema = 1; sourceRepo = $SourceRepoUrl; releases = $releases }
$json = ConvertTo-Json -InputObject $manifest -Depth 12
[System.IO.File]::WriteAllText($manifestPath, $json, (New-Object System.Text.UTF8Encoding $false))

foreach ($page in @("index.html", "flasher.js", "style.css")) {
    Copy-Item (Join-Path $PSScriptRoot $page) -Destination $Site -Force
}
# Serve the files as they are; Pages would otherwise run them through Jekyll.
$noJekyll = Join-Path $Site ".nojekyll"
if (-not (Test-Path $noJekyll)) { New-Item -ItemType File $noJekyll | Out-Null }

Write-Host ""
Write-Host "Staged build $build in $Site."
Write-Host "Try it locally:  cd $Site; python -m http.server 8000   then open http://localhost:8000 in Chrome or Edge"
if (-not $Bench) {
    Write-Host "Then commit and push the site (see README.md)."
}
