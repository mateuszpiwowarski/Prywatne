param(
    [string]$Environment = "esp32-wroom"
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

& $platformioExe run -e $Environment
exit $LASTEXITCODE
