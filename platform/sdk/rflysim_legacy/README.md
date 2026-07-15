# RflySim Legacy SDK Assets

This folder stores the local copies of the RflySim helper files needed by the
verification platform. Runtime code should import these files from this project,
not from older experiments or RflySim example folders.

Current copied asset:

- `PX4MavCtrlV4.py`: MAVLink/PX4 controller used by the verification `dt` backend.
- `EarthModel.py`: coordinate and earth-model helpers imported by `PX4MavCtrlV4.py`.

Keep this folder focused on shared RflySim integration code. Scenario execution, action mapping, and platform orchestration belong in `platform/verification`.
