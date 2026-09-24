# Flash every Heltec radio plugged into this PC with the build that is already made.
#
# Building is the slow part - a V4's first build compiles all of Meshtastic -
# and `pio run -t upload` rescans the whole tree before every flash. The same
# image goes on every board of a type, so build once and then write the
# finished file to each board with esptool. `-t nobuild` would be the pio way,
# but Meshtastic's bin/platformio-custom.py fails without a build ("Import of
# non-existent variable 'projenv'").
#
# What is written is what pio's upload writes for an update: the app at app0's
# offset (read from the build's partitions.bin) and boot_app0.bin over otadata,
# so the board boots app0 even after an OTA left it pointing at app1. NVS is
# left alone, so settings and Bluetooth pairing survive.
#
# Which build a port gets is decided by asking the chip, not by the USB ID. A
# USB vendor ID only says Silicon Labs or Espressif made the USB bridge, and
# every other ESP32 board on the bench has one of the same two; guessing from
# it would put a Heltec image on whatever else happened to be plugged in.
# esptool reads the chip itself:
#
#   Heltec V3  ESP32-S3,  8 MB flash, no PSRAM   -> heltec-v3
#   Heltec V4  ESP32-S3, 16 MB flash, 2 MB PSRAM -> heltec-v4
#
# Anything else is skipped. The plan is shown and nothing is flashed until you
# say yes. -Map overrides the detection for a port you know better than it.
#
#   .\flash-all.ps1                         build if stale, check chips, flash
#   .\flash-all.ps1 -NoBuild                flash what is built
#   .\flash-all.ps1 -Ports COM39            only these ports
#   .\flash-all.ps1 -Map COM39=heltec-v4    say what a port is
#   .\flash-all.ps1 -Yes                    no confirmation prompt

param(
    [string[]]$Ports,
    [string[]]$Map,
    [switch]$NoBuild,
    [switch]$Yes,
    [string]$Checkout = "C:\Projects\meshtastic-firmware"
)

$ErrorActionPreference = "Stop"
# The builds use C:\Projects\pio-clean, not ~/.platformio; take the tools from
# the same core so a flash never wakes the other one up.
if (-not $env:PLATFORMIO_CORE_DIR) { $env:PLATFORMIO_CORE_DIR = "C:\Projects\pio-clean" }
$core = $env:PLATFORMIO_CORE_DIR
$python = Join-Path $core "penv\Scripts\python.exe"
$esptool = Join-Path $core "packages\tool-esptoolpy\esptool.py"
$bootApp0 = Join-Path $core "packages\framework-arduinoespressif32\tools\partitions\boot_app0.bin"

# The finished image of one env and where it goes, from the build folder.
function Image-For([string]$envName) {
    $dir = Join-Path $Checkout ".pio\build\$envName"
    $app = Get-ChildItem (Join-Path $dir "firmware-*.bin") -ErrorAction SilentlyContinue |
        Where-Object { $_.Name -notmatch '\.factory\.bin$' } | Select-Object -First 1
    $table = Join-Path $dir "partitions.bin"
    if (-not $app -or -not (Test-Path $table)) { throw "no finished $envName build in $dir" }

    # 32-byte entries: magic AA 50, type, subtype, offset, size, name. Type 0 is
    # an app, subtype 0x10 is ota_0 (app0), 0x00 a plain factory app; 1/0 is otadata.
    $bytes = [IO.File]::ReadAllBytes($table)
    $appOffset = $null; $otaOffset = $null
    for ($i = 0; $i + 32 -le $bytes.Length; $i += 32) {
        if ($bytes[$i] -ne 0xAA -or $bytes[$i + 1] -ne 0x50) { break }
        $type = $bytes[$i + 2]; $sub = $bytes[$i + 3]
        $offset = [BitConverter]::ToUInt32($bytes, $i + 4)
        if ($type -eq 0 -and ($sub -eq 0x10 -or $sub -eq 0x00) -and $null -eq $appOffset) { $appOffset = $offset }
        if ($type -eq 1 -and $sub -eq 0x00) { $otaOffset = $offset }
    }
    if ($null -eq $appOffset) { throw "no app partition in $table" }
    return [pscustomobject]@{ App = $app.FullName; Built = $app.LastWriteTime; AppOffset = $appOffset; OtaOffset = $otaOffset }
}

$manual = @{}
foreach ($m in $Map) {
    $k, $v = $m -split '=', 2
    $manual[$k.Trim().ToUpper()] = $v.Trim()
}

# Ask the chip. Returns the env name, or $null with the reason printed.
function Env-For([string]$port) {
    if ($manual.ContainsKey($port)) { return $manual[$port] }
    # Continue, not Stop: Windows PowerShell turns any stderr line from a
    # native program into a terminating error once it is redirected.
    $ErrorActionPreference = "Continue"
    $out = & $python $esptool --port $port --after hard_reset flash_id 2>&1 | Out-String
    if ($LASTEXITCODE -ne 0) { Write-Host "  ${port}: esptool could not read the chip, skipping"; return $null }
    $isS3 = $out -match 'ESP32-S3'
    $flash = [regex]::Match($out, 'Detected flash size:\s*(\d+)MB').Groups[1].Value
    $psram = [regex]::Match($out, 'Embedded PSRAM\s*(\d+)MB').Groups[1].Value
    if ($isS3 -and $flash -eq '8' -and -not $psram) { return 'heltec-v3' }
    if ($isS3 -and $flash -eq '16' -and $psram -eq '2') { return 'heltec-v4' }
    Write-Host "  ${port}: not a Heltec V3 or V4 (S3=$isS3, flash=${flash}MB, PSRAM=${psram}MB), skipping"
    return $null
}

# Every serial port behind an Espressif or Silicon Labs USB bridge is a
# candidate; the chip check above decides which are Heltecs.
$candidates = Get-CimInstance Win32_PnPEntity |
    Where-Object { $_.Name -match '\((COM\d+)\)' -and $_.DeviceID -match 'VID_(10C4|303A)' } |
    ForEach-Object { [regex]::Match($_.Name, '\((COM\d+)\)').Groups[1].Value } |
    Where-Object { -not $Ports -or $Ports -contains $_ } |
    Sort-Object -Unique

if (-not $candidates) { Write-Host "No ESP32 boards found on USB."; exit 1 }

Write-Host "Checking chips..."
$boards = @()
foreach ($port in $candidates) {
    $envName = Env-For $port
    if ($envName) { $boards += [pscustomobject]@{ Port = $port; Env = $envName } }
}
if (-not $boards) { Write-Host "No Heltec V3 or V4 found."; exit 1 }
$boards | Format-Table -AutoSize | Out-String | Write-Host

if (-not $Yes) {
    $answer = Read-Host "Flash these $($boards.Count) board(s)? (y/n)"
    if ($answer -notmatch '^(y|yes)$') { Write-Host "Nothing flashed."; exit 0 }
}

Push-Location $Checkout
try {
    # Build each needed type once. pio skips the compile when nothing changed,
    # so this is quick when the image is already current.
    if (-not $NoBuild) {
        foreach ($envName in ($boards.Env | Sort-Object -Unique)) {
            Write-Host "== building $envName"
            pio run -e $envName
            if ($LASTEXITCODE -ne 0) { throw "build failed for $envName" }
        }
    }

    $images = @{}
    foreach ($envName in ($boards.Env | Sort-Object -Unique)) {
        $images[$envName] = Image-For $envName
        Write-Host "== $envName image: $(Split-Path -Leaf $images[$envName].App), built $($images[$envName].Built)"
    }

    # esptool prints warnings on stderr, which Windows PowerShell would turn
    # into a terminating error under Stop; its exit code is what counts.
    $ErrorActionPreference = "Continue"
    $failed = @()
    foreach ($b in $boards) {
        $img = $images[$b.Env]
        Write-Host "== flashing $($b.Env) on $($b.Port)"
        $writes = @(('0x{0:X}' -f $img.AppOffset), $img.App)
        if ($null -ne $img.OtaOffset) { $writes += @(('0x{0:X}' -f $img.OtaOffset), $bootApp0) }
        & $python $esptool --chip esp32s3 --port $b.Port --baud 921600 write_flash @writes
        if ($LASTEXITCODE -ne 0) { $failed += $b.Port }
    }
} finally {
    Pop-Location
}

if ($failed) { Write-Host "FAILED: $($failed -join ', ')"; exit 1 }
Write-Host "All $($boards.Count) board(s) flashed."
