# Legacy entry point; keep one compiler/argument implementation in run.bat.
param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]] $ProgramArguments
)
$ErrorActionPreference = 'Stop'
$launcher = Join-Path (Split-Path $PSScriptRoot -Parent) 'run.bat'
& $launcher @ProgramArguments
exit $LASTEXITCODE
