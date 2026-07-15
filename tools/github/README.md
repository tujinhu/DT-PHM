# GitHub Management Scripts

Run these BAT files manually from Windows.

0. `00_github_menu.bat`
   - One-click menu for the three workflows below.

1. `01_init_and_push_to_github.bat`
   - Use on the development PC for the first GitHub upload.
   - It initializes git, sets `origin`, commits, and pushes.

2. `02_sync_push_updates.bat`
   - Use on the development PC for later updates.
   - It pulls with rebase, commits local changes, and pushes.

3. `03_onboard_clone_or_update.bat`
   - Use on the onboard computer.
   - It clones the project if missing, or updates it if already cloned.

4. `04_auth_and_push_https.bat`
   - Use when GitHub reports `Invalid username or token`.
   - It tries GitHub CLI or Git Credential Manager browser login, then pushes.

Recommended flow:

```bat
tools\github\00_github_menu.bat
tools\github\01_init_and_push_to_github.bat
tools\github\03_onboard_clone_or_update.bat
tools\github\02_sync_push_updates.bat
tools\github\04_auth_and_push_https.bat
```

The project `.gitignore` excludes generated logs, reports, cache files, and
Excel outputs so GitHub stores source/configuration rather than runtime data.
