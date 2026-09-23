# Flash every Heltec radio plugged into this PC with the build that is already made.
#
# Building is the slow part - a V4's first build compiles all of Meshtastic -
# and `pio run -t upload` checks and relinks before every flash. The same image
# goes on every board of a type, so build once and then flash the finished
# file to each board with `-t nobuild`, which is exactly pio's own upload (same
# offsets, same bootloader, NVS left alone so settings and Bluetooth pairing
# survive) minus the compiler.
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
$python = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\python.exe"
$esptool = Join-Path $env:USERPROFILE ".platformio\packages\tool-esptoolpy\esptool.py"

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

    $failed = @()
    foreach ($b in $boards) {
        Write-Host "== flashing $($b.Env) on $($b.Port)"
        pio run -e $b.Env -t nobuild -t upload --upload-port $b.Port
        if ($LASTEXITCODE -ne 0) { $failed += $b.Port }
    }
} finally {
    Pop-Location
}

if ($failed) { Write-Host "FAILED: $($failed -join ', ')"; exit 1 }
Write-Host "All $($boards.Count) board(s) flashed."
