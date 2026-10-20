# Run from any directory; all paths below are relative to the project root.
param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]] $ProgramArguments
)
$ErrorActionPreference = 'Stop'
Push-Location (Split-Path $PSScriptRoot -Parent)
try {
    [Console]::OutputEncoding = [System.Text.UTF8Encoding]::new()
    chcp.com 65001 | Out-Null

    $mujocoDirectory = Join-Path (Get-Location) 'build/deps/mujoco'
    if (!(Test-Path "$mujocoDirectory/mujoco.dll")) {
        throw 'MuJoCo missing: expected build/deps/mujoco/mujoco.dll'
    }

    # Use the runtime DLLs belonging to the compiler, ahead of Conda DLLs.
    $compiler = (Get-Command g++ -ErrorAction Stop).Source
    $compilerDirectory = Split-Path $compiler -Parent
    $compilerRoot = Split-Path $compilerDirectory -Parent
    $previousPath = $env:PATH
    $env:PATH = "$mujocoDirectory;$compilerDirectory;$env:PATH"
    try {
        & $compiler -std=c++17 -Wall -Wextra -pedantic `
            -finput-charset=UTF-8 -fexec-charset=UTF-8 `
            -I include -I "$mujocoDirectory/include" -I "$compilerRoot/include" `
            src/main.cpp src/viewer.cpp src/mujoco_interface.cpp src/logger.cpp src/dynamics.cpp src/linearize.cpp "$mujocoDirectory/mujoco.dll" `
            -o build/double_pendulum.exe -L "$compilerRoot/lib" -lglfw3 -lopengl32 -lgdi32
        if ($LASTEXITCODE -ne 0) { throw 'Compilation failed' }

        & ./build/double_pendulum.exe @ProgramArguments
        if ($LASTEXITCODE -ne 0) { throw "Program exited with code $LASTEXITCODE" }
    } finally {
        $env:PATH = $previousPath
    }
} finally {
    Pop-Location
}
