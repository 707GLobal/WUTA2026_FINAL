param([string]$Port = 'COM3', [ValidateSet('native','bridge')][string]$Mode = 'native')
$ErrorActionPreference = 'Stop'
$radioPython = Join-Path $PSScriptRoot '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $radioPython)) { throw 'Run setup_windows.ps1 first' }
& $radioPython -m fsd_radio.cli --config "$PSScriptRoot\radio.yaml" pc --port $Port --mode $Mode
exit $LASTEXITCODE
