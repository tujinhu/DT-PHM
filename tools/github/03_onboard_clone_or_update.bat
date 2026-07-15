@echo off
setlocal EnableExtensions EnableDelayedExpansion

echo ============================================================
echo  DT-enabled PHM - onboard clone or update
echo ============================================================
echo  Run this script on the onboard computer.
echo  It clones the GitHub repository if the target folder does not exist,
echo  or pulls the latest version if the folder already exists.
echo.

where git >nul 2>nul
if errorlevel 1 (
    echo [ERROR] git was not found on this machine.
    pause
    exit /b 1
)

set "DEFAULT_TARGET=D:\TJH\Platform\DT-enabled PHM"
set "DEFAULT_BRANCH=main"

set /p REPO_URL=GitHub repository URL, for example https://github.com/USER/REPO.git: 
if "!REPO_URL!"=="" (
    echo [ERROR] Repository URL is required.
    pause
    exit /b 1
)

set /p TARGET_DIR=Local target folder [!DEFAULT_TARGET!]: 
if "!TARGET_DIR!"=="" set "TARGET_DIR=!DEFAULT_TARGET!"

set /p BRANCH=Branch name [main]: 
if "!BRANCH!"=="" set "BRANCH=!DEFAULT_BRANCH!"

if exist "!TARGET_DIR!\.git" (
    echo.
    echo [INFO] Existing repository found: !TARGET_DIR!
    cd /d "!TARGET_DIR!"
    if errorlevel 1 goto :fail
    git fetch origin
    if errorlevel 1 goto :fail
    git checkout "!BRANCH!"
    if errorlevel 1 goto :fail
    git pull --rebase origin "!BRANCH!"
    if errorlevel 1 goto :fail
) else (
    if exist "!TARGET_DIR!" (
        echo [ERROR] Target folder exists but is not a git repository:
        echo         !TARGET_DIR!
        echo         Choose an empty/new folder, or delete/backup the existing folder manually.
        pause
        exit /b 1
    )
    echo.
    echo [INFO] Cloning repository...
    git clone -b "!BRANCH!" "!REPO_URL!" "!TARGET_DIR!"
    if errorlevel 1 goto :fail
)

echo.
echo [OK] Onboard project is ready:
echo      !TARGET_DIR!
pause
exit /b 0

:fail
echo.
echo [ERROR] Clone/update failed. Please check network, credentials, branch name, and repository URL.
pause
exit /b 1
