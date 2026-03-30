@echo off
REM Build and run JoltC test suite (Win-x64, Release)
pushd "%~dp0"

echo === Configuring CMake (x64) ===
cmake -B build -A x64
if %ERRORLEVEL% neq 0 (
    echo CMake configure FAILED
    popd
    exit /b 1
)

echo === Building (Release) ===
cmake --build build --config Release
if %ERRORLEVEL% neq 0 (
    echo Build FAILED
    popd
    exit /b 1
)

echo === Running tests ===
build\Release\JoltC_Tests.exe
set RESULT=%ERRORLEVEL%

popd
exit /b %RESULT%
