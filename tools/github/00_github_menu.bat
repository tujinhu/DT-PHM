@echo off
setlocal EnableExtensions EnableDelayedExpansion

cd /d "%~dp0"

:menu
cls
echo ============================================================
echo  DT-enabled PHM - GitHub one-click menu
echo ============================================================
echo.
echo  1. First upload this project to GitHub
echo  2. Sync local updates to GitHub
echo  3. Onboard clone or update from GitHub
echo  0. Exit
echo.
set /p CHOICE=Select an option: 

if "!CHOICE!"=="1" (
    call "%~dp001_init_and_push_to_github.bat"
    goto :menu
)
if "!CHOICE!"=="2" (
    call "%~dp002_sync_push_updates.bat"
    goto :menu
)
if "!CHOICE!"=="3" (
    call "%~dp003_onboard_clone_or_update.bat"
    goto :menu
)
if "!CHOICE!"=="0" exit /b 0

echo.
echo [WARN] Invalid option.
pause
goto :menu
