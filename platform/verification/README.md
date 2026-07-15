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

Each run writes one `.xlsx` workbook to `platform/verification/log`, with one sheet per vehicle.

## Real verification

```powershell
python platform\verification\dt-dt.py
```

The default configs now point to project-local copies of the needed legacy assets:

- `platform/sdk/toolchain/SITLRun.bat` on Windows
- `platform/sdk/toolchain/SITLRun.sh` on Linux
- `platform/sdk/rflysim_legacy/PX4MavCtrlV4.py`

Adjust the config files if your RflySim installation paths change.
