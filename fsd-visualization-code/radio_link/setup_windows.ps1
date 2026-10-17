param([string]$Python = 'python')
$ErrorActionPreference = 'Stop'
& $Python -m venv "$PSScriptRoot\.venv"
if ($LASTEXITCODE -ne 0) { throw 'Python venv creation failed' }
& "$PSScriptRoot\.venv\Scripts\python.exe" -m pip install -e "${PSScriptRoot}[pc]"
if ($LASTEXITCODE -ne 0) { throw 'Radio dependency installation failed' }
Write-Output 'Ready. Run .\start_windows.ps1 and connect Foxglove to ws://127.0.0.1:8765'
