# RflySim Verification Platform

This folder is the semantic-Scenario verification layer for DT-enabled PHM.

## Design

- `configs/*.json` contains both semantic scenario data (`case_id`, `name`, `timeline`) and runtime topology (RflySim toolchain, vehicle count, and backend types).
- `dt-dt.py` runs two full RflySim/SITL digital twins.
- `generic.py` can run configurable DT-only topologies from config.
- The verification business runner lives in `verification_runner.py`; shared adapters live under `platform/sdk`.

## Backend choice

- Use `dt` for verification that must replace the future application/real-aircraft path. It launches or connects to the full RflySim toolchain (`SITLRun.bat`, PX4, CopterSim, RflySim3D).

## Dry-run validation

```powershell
python platform\verification\dt-dt.py --dry-run
python platform\verification\generic.py --dry-run
```

## Real-time recording

Run configs can enable state recording. The JSON files use `_comment` and
`_options` fields as standard-JSON annotations for external users:

```json
"recording": {
  "enabled": true,
  "sample_hz": 50,
  "output_dir": "../log",
  "fields": [
    "uavAngEular",
    "uavAngRate",
    "uavPosNED",
    "uavVelNED",
    "uavAngQuatern",
    "uavAccB",
    "uavGyro",
    "uavMag",
    "uavVibr"
  ]
}
```

Each run writes one online `.xlsx` workbook to `platform/verification/log/online`,
with one sheet per vehicle.

DT-only verification can also archive PX4 offline ULog files:

```json
"offline_logs": {
  "enabled": true,
  "output_dir": "../log/offline",
  "group_by_run": true,
  "settle_s": 2.0
}
```

The default PX4 source path is:

- Windows: `C:/PX4PSP/Firmware/build/px4_sitl_default/instance_*/log/<date>/*.ulg`
- Linux: `~/PX4PSP/Firmware/build/px4_sitl_default/instance_*/log/<date>/*.ulg`

Set `offline_logs.firmware_dir`, `offline_logs.build_dir`, or
`offline_logs.date` in JSON when a run uses a non-default location.

## Real verification

```powershell
python platform\verification\dt-dt.py
```

The default configs now point to project-local copies of the needed legacy assets:

- `platform/sdk/toolchain/SITLRun.bat` on Windows
- `platform/sdk/toolchain/SITLRun.sh` on Linux
- `platform/sdk/rflysim_legacy/PX4MavCtrlV4.py`

Adjust the config files if your RflySim installation paths change.

Set the CopterSim model through JSON:

```json
"toolchain": {
  "env": {
    "RFV_DLL_MODEL": "FX120_model"
  }
}
```

This value is written into `DLLModel` in both `SITLRun.bat` and `SITLRun.sh`.
