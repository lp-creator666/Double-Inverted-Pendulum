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
    $eigenDirectory = $env:EIGEN3_INCLUDE_DIR
    if (!$eigenDirectory) {
        $eigenDirectory = Join-Path (Get-Location) 'build/deps/eigen-3.4.0'
    }
    if (!(Test-Path "$eigenDirectory/Eigen/SVD")) {
        throw 'Eigen missing: set EIGEN3_INCLUDE_DIR or install build/deps/eigen-3.4.0'
    }
    $previousPath = $env:PATH
    $env:PATH = "$mujocoDirectory;$compilerDirectory;$env:PATH"
    try {
        & $compiler -std=c++17 "-Wa,-mbig-obj" -Wall -Wextra -pedantic `
            -finput-charset=UTF-8 -fexec-charset=UTF-8 `
            -I "$eigenDirectory" -I include -I "$mujocoDirectory/include" -I "$compilerRoot/include" `
            src/main.cpp src/lqr.cpp src/lmpc.cpp src/model_diagnostics.cpp src/viewer.cpp src/mujoco_interface.cpp src/logger.cpp src/dynamics.cpp src/linearize.cpp src/controllability.cpp src/scaling.cpp "$mujocoDirectory/mujoco.dll" `
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
