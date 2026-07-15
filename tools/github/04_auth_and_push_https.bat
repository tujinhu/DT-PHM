@echo off
setlocal EnableExtensions EnableDelayedExpansion

cd /d "%~dp0..\.."

echo ============================================================
echo  DT-enabled PHM - GitHub HTTPS auth and push
echo ============================================================
echo  GitHub does not accept account passwords for git push.
echo  This script tries browser-based authentication first, then
echo  pushes the current branch to origin.
echo.

where git >nul 2>nul
if errorlevel 1 (
    echo [ERROR] git was not found. Please install Git for Windows first.
    pause
    exit /b 1
)

if not exist ".git" (
    echo [ERROR] This folder is not a git repository.
    pause
    exit /b 1
)

for /f "delims=" %%i in ('git branch --show-current') do set "BRANCH=%%i"
if "!BRANCH!"=="" set "BRANCH=main"

for /f "delims=" %%i in ('git remote get-url origin') do set "ORIGIN_URL=%%i"
if "!ORIGIN_URL!"=="" (
    echo [ERROR] origin remote is not configured.
    pause
    exit /b 1
)

echo [INFO] Branch: !BRANCH!
echo [INFO] Origin: !ORIGIN_URL!
echo.

git status --short
echo.
set /p COMMIT_PENDING=Commit pending local changes before push? [Y/n]: 
if /i not "!COMMIT_PENDING!"=="n" (
    set /p COMMIT_MSG=Commit message [Update GitHub scripts]: 
    if "!COMMIT_MSG!"=="" set "COMMIT_MSG=Update GitHub scripts"
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
)
echo.

where gh >nul 2>nul
if not errorlevel 1 (
    echo [INFO] GitHub CLI detected. Starting browser login...
    gh auth login -h github.com -p https -w
    if not errorlevel 1 (
        gh auth setup-git
        echo.
        echo [INFO] Pushing with GitHub CLI credentials...
        git push -u origin "!BRANCH!"
        if not errorlevel 1 goto :ok
    )
    echo [WARN] GitHub CLI authentication did not complete successfully.
    echo.
)

echo [INFO] Trying Git Credential Manager browser login...
git credential-manager github login
if errorlevel 1 (
    git credential-manager-core github login
)

echo.
echo [INFO] Pushing to origin/!BRANCH! ...
git push -u origin "!BRANCH!"
if not errorlevel 1 goto :ok

echo.
echo [ERROR] Push still failed.
echo.
echo  Manual fallback:
echo  1. Open GitHub - Settings - Developer settings - Personal access tokens.
echo  2. Create a token with repo permission.
echo  3. Run: git push -u origin !BRANCH!
echo  4. Username: your GitHub username
echo  5. Password: paste the token, not your account password
echo.
pause
exit /b 1

:ok
echo.
echo [OK] Authentication and push finished.
pause
exit /b 0

:fail
echo.
echo [ERROR] Local commit preparation failed.
pause
exit /b 1
