param([switch]$Quick)
$ErrorActionPreference = 'Stop'
Push-Location (Split-Path $PSScriptRoot -Parent)
$savedPath=$env:PATH
try {
    $compiler=(Get-Command g++ -ErrorAction Stop).Source
    $compilerDirectory=Split-Path $compiler -Parent
    $env:PATH="$PWD/build/deps/mujoco;$compilerDirectory;$env:PATH"
    $cmake=Join-Path $PWD 'build/deps/cmake/data/bin/cmake.exe'
    if(!(Test-Path -LiteralPath $cmake)) { $cmake=(Get-Command cmake -ErrorAction Stop).Source }
    $ctest=Join-Path (Split-Path $cmake -Parent) 'ctest.exe'
    & $cmake -S . -B build/svd-check -DCMAKE_BUILD_TYPE=Release
    if($LASTEXITCODE -ne 0) {throw 'CMake configuration failed'}
    & $cmake --build build/svd-check -j 2
    if($LASTEXITCODE -ne 0) {throw 'Build failed'}
    $testArguments=@('--test-dir','build/svd-check','--output-on-failure')
    if($Quick) {$testArguments+=@('-LE','acceptance')}
    & $ctest @testArguments
    if($LASTEXITCODE -ne 0) {throw 'Validation failed; see failed test output'}
} finally {
    $env:PATH=$savedPath
    Pop-Location
}
