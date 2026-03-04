# Modulux ESP32-S3 Server Launcher (PowerShell)
param([int]$Port = 8765)

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $scriptDir

$pythonExe = $null
$pyCmd = Get-Command py -ErrorAction SilentlyContinue
if ($pyCmd) {
    $pythonExe = 'py'
} else {
    $pythonCmd = Get-Command python -ErrorAction SilentlyContinue
    if ($pythonCmd) {
        $pythonExe = $pythonCmd.Source
    } elseif (Test-Path "$env:LocalAppData\Programs\Python\Python312\python.exe") {
        $pythonExe = "$env:LocalAppData\Programs\Python\Python312\python.exe"
    } elseif (Test-Path "$env:LocalAppData\Programs\Python\Python311\python.exe") {
        $pythonExe = "$env:LocalAppData\Programs\Python\Python311\python.exe"
    }
}

if (-not $pythonExe) {
    Write-Host '[ERROR] Python not found' -ForegroundColor Red
    exit 1
}

$env:MODULUX_PORT = [string]$Port

Write-Host '=================================================='
Write-Host '  Modulux ESP32-S3 Blockly IDE Server'
Write-Host ("  http://localhost:{0}" -f $Port)
Write-Host '=================================================='
Write-Host 'Press Ctrl+C to stop'
Write-Host ''

if ($pythonExe -eq 'py') {
    & py .\server.py
} else {
    & $pythonExe .\server.py
}

exit $LASTEXITCODE
