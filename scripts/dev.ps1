<#
.SYNOPSIS
    ADAPT-X developer task runner (Windows).

.EXAMPLE
    .\scripts\dev.ps1 install
    .\scripts\dev.ps1 test
    .\scripts\dev.ps1 check
#>
param(
    [Parameter(Mandatory = $true)]
    [ValidateSet('install', 'run', 'test', 'test-unit', 'test-integration',
                 'lint', 'format', 'format-check', 'typecheck', 'check',
                 'docker-build', 'docker-up')]
    [string]$Task
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$python = Join-Path $root '.venv\Scripts\python.exe'

if (-not (Test-Path $python)) {
    Write-Host "No virtual environment found. Creating one at $root\.venv" -ForegroundColor Yellow
    python -m venv (Join-Path $root '.venv')
}

Push-Location $root
try {
    switch ($Task) {
        'install'          { & $python -m pip install --upgrade pip; & $python -m pip install -e ".[dev]" }
        'run'              { & $python -m adaptx }
        'test'             { & $python -m pytest }
        'test-unit'        { & $python -m pytest tests/unit }
        'test-integration' { & $python -m pytest tests/integration }
        'lint'             { & $python -m ruff check . }
        'format'           { & $python -m ruff format . }
        'format-check'     { & $python -m ruff format --check . }
        'typecheck'        { & $python -m mypy }
        'check' {
            & $python -m ruff check .
            if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
            & $python -m ruff format --check .
            if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
            & $python -m mypy
            if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
            & $python -m pytest
        }
        'docker-build'     { docker compose build }
        'docker-up'        { docker compose up }
    }
    exit $LASTEXITCODE
}
finally {
    Pop-Location
}
