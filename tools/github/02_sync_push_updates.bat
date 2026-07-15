@echo off
setlocal EnableExtensions EnableDelayedExpansion

cd /d "%~dp0..\.."

echo ============================================================
echo  DT-enabled PHM - sync local updates to GitHub
echo ============================================================
echo.

where git >nul 2>nul
if errorlevel 1 (
    echo [ERROR] git was not found. Please install Git for Windows first.
    pause
    exit /b 1
)

if not exist ".git" (
    echo [ERROR] This folder is not a git repository. Run 01_init_and_push_to_github.bat first.
    pause
    exit /b 1
)

for /f "delims=" %%i in ('git branch --show-current') do set "BRANCH=%%i"
if "!BRANCH!"=="" set "BRANCH=main"

echo [INFO] Current branch: !BRANCH!
echo.
git status --short
echo.

set /p COMMIT_MSG=Commit message [Update project]: 
if "!COMMIT_MSG!"=="" set "COMMIT_MSG=Update project"

git add .
if errorlevel 1 goto :fail

echo [INFO] Removing local-only folders from git index if they were tracked...
git rm --cached -r --ignore-unmatch docs >nul 2>nul

git diff --cached --quiet
if errorlevel 1 (
    git commit -m "!COMMIT_MSG!"
    if errorlevel 1 goto :fail
) else (
    echo [INFO] No staged changes to commit.
)

set /p DO_PULL=Pull remote changes with rebase before pushing? [Y/n]: 
if /i not "!DO_PULL!"=="n" (
    git pull --rebase origin "!BRANCH!"
    if errorlevel 1 goto :fail
)

git push origin "!BRANCH!"
if errorlevel 1 goto :auth_fail

echo.
echo [OK] Sync push finished.
pause
exit /b 0

:fail
echo.
echo [ERROR] Sync failed. Resolve the issue above, then rerun this script.
pause
exit /b 1

:auth_fail
echo.
echo [ERROR] Push failed. If the message says "Invalid username or token",
echo         run tools\github\04_auth_and_push_https.bat or choose option 4
echo         in tools\github\00_github_menu.bat.
pause
exit /b 1
