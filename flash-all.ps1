# Flash every Heltec radio plugged into this PC with the build that is already made.
#
# Building is the slow part - a V4's first build compiles all of Meshtastic -
# and `pio run -t upload` checks and relinks before every flash. The same image
# goes on every board of a type, so build once and then flash the finished
# file to each board with `-t nobuild`, which is exactly pio's own upload (same
# offsets, same bootloader, NVS left alone so settings and Bluetooth pairing
# survive) minus the compiler.
#
# Board type comes from the USB bridge: a V3 talks through a CP210x
# (VID 10C4), a V4 through the ESP32-S3's own USB (VID 303A).
#
#   .\flash-all.ps1              build anything stale, then flash every board
#   .\flash-all.ps1 -NoBuild     flash what is built, whatever its age
#   .\flash-all.ps1 -Ports COM39 flash just these ports
#
# Run from anywhere; it works in C:\Projects\meshtastic-firmware.

param(
    [string[]]$Ports,
    [switch]$NoBuild,
    [string]$Checkout = "C:\Projects\meshtastic-firmware"
)

$ErrorActionPreference = "Stop"

function Env-For([string]$instanceId) {
    if ($instanceId -match 'VID_10C4') { return 'heltec-v3' }
    if ($instanceId -match 'VID_303A') { return 'heltec-v4' }
    return $null
}

# Every serial port with a Heltec behind it, and which build it needs.
$boards = @()
Get-CimInstance Win32_PnPEntity | Where-Object { $_.Name -match '\((COM\d+)\)' } | ForEach-Object {
    $port = [regex]::Match($_.Name, '\((COM\d+)\)').Groups[1].Value
    $envName = Env-For $_.DeviceID
    if ($envName -and (-not $Ports -or $Ports -contains $port)) {
        $boards += [pscustomobject]@{ Port = $port; Env = $envName }
    }
}

if (-not $boards) { Write-Host "No Heltec boards found on USB."; exit 1 }
$boards | Format-Table -AutoSize | Out-String | Write-Host

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
