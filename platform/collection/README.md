# RflySim Collection Platform

`collect.py` runs one unified real+DT online collection flow. The runner stays in
this folder because collection has different lifecycle rules from verification:
recording starts after `arm`, each timeline writes a separate workbook, and real
vehicle fault flags are cleared after every round.

The reusable pieces are inherited from `platform/sdk`:

- `backends/dt_vehicle.py` manages the healthy DT.
- `backends/real_vehicle.py` manages the ROS real-vehicle path.
- `core/logging.py` writes one xlsx workbook with one sheet per vehicle.
- `core/toolchain.py` selects `SITLRun.bat` on Windows and `SITLRun.sh` on Linux.

Dry-run validation:

```powershell
python platform\collection\collect.py --dry-run
```

Real collection:

```bash
python3 platform/collection/collect.py --config platform/collection/configs/real_dt_collection.json
```

For real flights, adjust the real vehicle `ip`, `port`, VRPN `rig_prefix` or
explicit topics in `configs/real_dt_collection.json`. Fault injection defaults
to `backend == "real"` so the DT remains a healthy baseline unless a timeline
explicitly targets the DT.
