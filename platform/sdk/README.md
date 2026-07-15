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
