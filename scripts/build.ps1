param([string]$EfinityRoot = 'D:/Efinity/2025.1')
$ErrorActionPreference = 'Stop'
$projectDir = Join-Path $PSScriptRoot '../project'
$runner = Join-Path $EfinityRoot 'bin/efx_run.bat'
if (-not (Test-Path -LiteralPath $runner)) { throw 'Set -EfinityRoot to your Efinity installation.' }
Push-Location -LiteralPath $projectDir
try {
    foreach ($phase in @('interface', 'map', 'pnr', 'pgm')) {
        & $runner 'Ti60_Demo.xml' --flow $phase --timing_model I3
        if ($LASTEXITCODE -ne 0) { throw "Efinity phase failed: $phase" }
    }
} finally { Pop-Location }
