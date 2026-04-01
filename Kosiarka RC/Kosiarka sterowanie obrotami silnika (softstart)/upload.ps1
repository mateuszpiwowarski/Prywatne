param(
    [string]$Environment = "esp32-wroom",
    [string]$Port = ""
)

$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$defaultPlatformIoCoreDir = Join-Path $env:LOCALAPPDATA "PlatformIO-KosiarkaSoftstart"
if (-not $env:PLATFORMIO_CORE_DIR) {
    $env:PLATFORMIO_CORE_DIR = $defaultPlatformIoCoreDir
}
$platformioExe = "C:\Users\mpiwowarski\.platformio\penv\Scripts\platformio.exe"

if (-not (Test-Path $platformioExe)) {
    throw "Nie znaleziono PlatformIO: $platformioExe"
}

if (-not $Port) {
    try {
        $devicesJson = & $platformioExe device list --json-output
        if ($LASTEXITCODE -eq 0 -and $devicesJson) {
            $devices = $devicesJson | ConvertFrom-Json
            $espDevice = $devices | Where-Object {
                $_.hwid -match "VID:PID=(10C4:EA60|303A:1001|1A86:7523)"
            } | Select-Object -First 1
            if ($espDevice) {
                $Port = $espDevice.port
                Write-Host "Wykryto ESP32 na porcie $Port"
            }
        }
    }
    catch {
        Write-Host "Nie udalo sie automatycznie wykryc portu, PlatformIO sprobuje samo."
    }
}

$args = @("run", "-e", $Environment, "-t", "upload")
if ($Port) {
    $args += @("--upload-port", $Port)
}

& $platformioExe @args
exit $LASTEXITCODE
