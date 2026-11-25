# TSID 1.7.1 patches

`scripts/setup_rosenv.sh` and `scripts/setup_wbc.sh` apply every `tsid-1.7.1-*.patch`
in this directory before building TSID; `walking_controller` and `wbc_control_node` need them.

| Patch | Fix |
|---|---|
| `tsid-1.7.1-rotor-setters.patch` | NumPy arrays can reach the rotor inertia / gear ratio setters |
| `tsid-1.7.1-soft-contact-removal.patch` | Removing a soft contact no longer underflows the hard equality count |
| `tsid-1.7.1-actuation-equality-binding.patch` | Python binding for `TaskActuationEquality` / `addActuationTask` |

Details: [deprecated/wbc_prototype/README.md](../../deprecated/wbc_prototype/README.md).
