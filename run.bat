@echo off
setlocal
set "PENDULUM_RUN_FILE=%~f0"
set "PENDULUM_RUN_ARGS=%*"
powershell.exe -NoProfile -ExecutionPolicy Bypass -Command "$text=[IO.File]::ReadAllText($env:PENDULUM_RUN_FILE); & ([scriptblock]::Create(($text -split '(?m)^# POWERSHELL-BEGIN\r?$',2)[1]))"
set "RUN_EXIT_CODE=%ERRORLEVEL%"
if not "%RUN_EXIT_CODE%"=="0" pause
endlocal & exit /b %RUN_EXIT_CODE%
# POWERSHELL-BEGIN
$ErrorActionPreference = 'Stop'
try {
    $root = [IO.Path]::GetDirectoryName($env:PENDULUM_RUN_FILE)
    Set-Location -LiteralPath $root
    [Console]::OutputEncoding = [System.Text.UTF8Encoding]::new()
    $culture = [Globalization.CultureInfo]::InvariantCulture
    # Preserve quoted arguments, including output paths containing spaces.
    $tokens = @([regex]::Matches($env:PENDULUM_RUN_ARGS, '(?:[^\s"]|"[^"]*")+') | ForEach-Object { $_.Value.Replace('"', '') })
    $arguments = @()
    for ($i = 0; $i -lt $tokens.Count; $i++) {
        $arg = $tokens[$i]
        if ($arg -eq '--q1-deg' -or $arg -eq '--q2-deg') {
            if ($i + 1 -ge $tokens.Count) { throw ('Missing degree value for ' + $arg) }
            $value = [double]::Parse($tokens[++$i], $culture)
            if ([double]::IsNaN($value) -or [double]::IsInfinity($value)) { throw 'Angle must be finite' }
            $radians = $value * ([Math]::PI / 180.0)
            if ([double]::IsInfinity($radians)) { throw 'Angle is too large' }
            $arguments += ($arg -replace '-deg$', '')
            $arguments += $radians.ToString('R', $culture)
        } else {
            $arguments += $arg
        }
    }
    if ($arguments -contains '--help') {
        Write-Output 'run.bat --lqr --q1-deg 0.5 --q2-deg 0'
        Write-Output 'run.bat --lmpc --q1-deg 1 --q2-deg -1'
        Write-Output 'run.bat --sqp --headless --duration 1 --q1-deg 1 --q2-deg 0'
        Write-Output 'run.bat --swingup --headless --duration 5 --q1-deg 180 --q2-deg 0'
        Write-Output 'run.bat --plan config/plans/sqp_swingup.csv --duration 10'
        Write-Output 'run.bat --track-plan config/plans/sqp_swingup.csv --duration 10 --push-force 1 --push-start 1 --push-duration 0.1'
        Write-Output 'run.bat --online-plan config/plans/sqp_swingup.csv --duration 10'
        Write-Output 'run.bat --track-plan config/scenarios/down_q2_90.csv --duration 10'
        Write-Output '--online-plan: seeded shrinking-horizon SQP; simulation may run slower than real time.'
        Write-Output '--solve-budget SECONDS: soft budget, default 0.5; missed 0.08 s control deadlines are logged separately.'
        Write-Output '--track-plan: offline SQP reference + time-varying LQR feedback, then balance LQR.'
        Write-Output 'Plan initial errors: --plan-q1-offset-deg 0.1 --plan-q2-offset-deg -0.1'
        Write-Output '--plan: offline SQP feedforward, then measured-state LQR; initial state comes from CSV.'
        Write-Output 'Angles: --q1-deg/--q2-deg use degrees; --q1/--q2 still use radians.'
        Write-Output 'q2 is relative to pole 1. Pole 2 absolute angle = q1 + q2.'
        Write-Output 'Other options: --headless --duration SECONDS --dq1 RAD_S --dq2 RAD_S --output FILE --force N'
        exit 0
    }
    $mujoco = Join-Path $root 'build/deps/mujoco'
    if (!(Test-Path "$mujoco/mujoco.dll")) { throw 'MuJoCo missing: build/deps/mujoco/mujoco.dll' }
    $compiler = (Get-Command g++ -ErrorAction Stop).Source
    $compilerDirectory = Split-Path $compiler -Parent
    $compilerRoot = Split-Path $compilerDirectory -Parent
    $eigen = $env:EIGEN3_INCLUDE_DIR
    if (!$eigen) { $eigen = Join-Path $root 'build/deps/eigen-3.4.0' }
    if (!(Test-Path "$eigen/Eigen/SVD")) { throw 'Eigen missing: set EIGEN3_INCLUDE_DIR' }
    $env:PATH = "$mujoco;$compilerDirectory;$env:PATH"
    # Discover sources after folder moves; do not depend on stale run.ps1 paths.
    $sources = @(Get-ChildItem -LiteralPath (Join-Path $root 'src') -Recurse -Filter '*.cpp' | Sort-Object FullName | ForEach-Object { $_.FullName })
    # Energy mode remains available; multiple-shooting helpers belong to tests only.
    $sources += (Join-Path $root 'research/swingup_controller.cpp')
    # Cache only compilation, never simulation results. Header edits and source
    # additions/removals also invalidate this fingerprint.
    $inputs=@(Get-Item -LiteralPath $env:PENDULUM_RUN_FILE)
    $inputs+=@(Get-ChildItem -LiteralPath (Join-Path $root 'src') -Recurse -File)
    $inputs+=@(Get-ChildItem -LiteralPath (Join-Path $root 'include') -Recurse -File)
    $inputs+=@(Get-ChildItem -LiteralPath (Join-Path $root 'research') -Recurse -File)
    $inputs+=@(Get-Item -LiteralPath "$mujoco/mujoco.dll", "$eigen/Eigen/Core", $compiler)
    $fingerprint=(@($compiler,$eigen,$compilerRoot) + @($inputs | Sort-Object FullName | ForEach-Object {
        $_.FullName+'|'+$_.Length+'|'+$_.LastWriteTimeUtc.Ticks
    })) -join "`n"
    $signature=Join-Path $root 'build/run.signature'
    $previous=if(Test-Path -LiteralPath $signature){[IO.File]::ReadAllText($signature)}else{''}
    if(!(Test-Path -LiteralPath 'build/double_pendulum.exe') -or $fingerprint -ne $previous) {
        Write-Output 'Building C++ controllers...'
        & $compiler -std=c++17 -O2 '-Wa,-mbig-obj' -Wall -Wextra -pedantic -finput-charset=UTF-8 -fexec-charset=UTF-8 -isystem $eigen -I include -I research/include -I "$mujoco/include" -I "$compilerRoot/include" @sources "$mujoco/mujoco.dll" -o build/double_pendulum.exe -L "$compilerRoot/lib" -lglfw3 -lopengl32 -lgdi32
        if ($LASTEXITCODE -ne 0) { throw 'Compilation failed' }
        [IO.File]::WriteAllText($signature,$fingerprint)
    }
    & ./build/double_pendulum.exe @arguments
    exit $LASTEXITCODE
} catch {
    Write-Error $_ -ErrorAction Continue
    exit 1
}
