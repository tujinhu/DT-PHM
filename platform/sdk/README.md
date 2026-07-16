# Platform SDK

This folder contains reusable platform code shared by verification, collection,
and future application runtimes.

- `core/`: scenario loading, logging, recording, trajectory helpers, and toolchain launch logic.
- `backends/`: vehicle backend implementations and dry-run backend.
- `toolchain/`: project-local RflySim toolchain launch assets.
- `rflysim_legacy/`: copied RflySim Python helper APIs required by the DT backend.

Platform entry points should add this folder to `sys.path` before importing
`core.*` or `backends.*`. Business runners stay in their own folders, for
example `platform/verification/verification_runner.py` and
`platform/collection/collection_runner.py`.

## Cross-platform toolchain

`core/toolchain.py` selects the launch script by host OS:

- Windows: `platform/sdk/toolchain/SITLRun.bat`
- Linux: `platform/sdk/toolchain/SITLRun.sh`

Configure both paths in JSON:

```json
"toolchain": {
  "enabled": true,
  "scripts": {
    "windows": "../../sdk/toolchain/SITLRun.bat",
    "linux": "../../sdk/toolchain/SITLRun.sh"
  },
  "env": {
    "RFV_VEHICLE_NUM": "2",
    "RFV_START_INDEX": "1",
    "RFV_DLL_MODEL": "FX120_model"
  }
}
```

On Linux, `SITLRun.sh` starts the PX4 SITL side. Set `PX4_FIRMWARE_DIR` when
the firmware tree is not available at `platform/px4/Firmware` or
`$PSP_PATH_LINUX/Firmware`.

`core/logging.py` provides both online xlsx recording and offline PX4 ULog
archiving. `PX4OfflineLogCollector` automatically filters configured vehicles
to DT backends, so mixed `dt + real` runs archive only the DT logs.
