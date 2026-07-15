@echo off
setlocal EnableExtensions EnableDelayedExpansion

cd /d "%~dp0..\.."

echo ============================================================
echo  DT-enabled PHM - first GitHub upload
echo ============================================================
echo  This script initializes git when needed, commits the project,
echo  sets the GitHub remote, and pushes to the selected branch.
echo.

where git >nul 2>nul
if errorlevel 1 (
    echo [ERROR] git was not found. Please install Git for Windows first.
    pause
    exit /b 1
)

set "DEFAULT_BRANCH=main"
set /p BRANCH=Branch name [main]: 
if "!BRANCH!"=="" set "BRANCH=!DEFAULT_BRANCH!"

if not exist ".git" (
    echo [INFO] Initializing git repository...
    git init
    if errorlevel 1 goto :fail
)

git branch -M "!BRANCH!"
if errorlevel 1 goto :fail

git remote get-url origin >nul 2>nul
if errorlevel 1 (
    echo.
    set /p REPO_URL=GitHub repository URL, for example https://github.com/USER/REPO.git: 
    if "!REPO_URL!"=="" (
        echo [ERROR] Repository URL is required.
        pause
        exit /b 1
    )
    git remote add origin "!REPO_URL!"
    if errorlevel 1 goto :fail
) else (
    for /f "delims=" %%i in ('git remote get-url origin') do set "OLD_URL=%%i"
    echo [INFO] Current origin: !OLD_URL!
    set /p REPO_URL=New GitHub URL, or press Enter to keep current origin: 
    if not "!REPO_URL!"=="" (
        git remote set-url origin "!REPO_URL!"
        if errorlevel 1 goto :fail
    )
)

echo.
git status --short
echo.
set /p COMMIT_MSG=Commit message [Initial platform scaffold]: 
if "!COMMIT_MSG!"=="" set "COMMIT_MSG=Initial platform scaffold"

git add .
if errorlevel 1 goto :fail

git diff --cached --quiet
if errorlevel 1 (
    git commit -m "!COMMIT_MSG!"
    if errorlevel 1 goto :fail
) else (
    echo [INFO] No staged changes to commit.
)

echo.
echo [INFO] Pushing to origin/!BRANCH! ...
git push -u origin "!BRANCH!"
if errorlevel 1 goto :fail

echo.
echo [OK] Upload finished.
pause
exit /b 0

:fail
echo.
echo [ERROR] Command failed. Please check the message above.
pause
exit /b 1
