# Contributing

Thanks for your interest in contributing. This repository contains:
- `crates/qmi8658`: the driver crate
- `apps/qa-runner`: hardware test runner for ESP32-S3
- `apps/imu-example`: FIFO-based example app

## Development Setup
- Rust toolchain: `esp` (see `apps/*/rust-toolchain.toml`)
- Target: `xtensa-esp32s3-none-elf`
- Host build for the driver crate works with a stable toolchain

## Building
Driver crate:
```bash
cargo build -p ph-qmi8658
```

QA runner:
```bash
cd apps/qa-runner
cargo build
```

Example app:
```bash
cd apps/imu-example
cargo build
```

## Running on Hardware
The apps are configured for ESP32-S3 using `espflash`.

```bash
espflash flash --monitor --log-format defmt
```

## Documentation
- Driver overview and flows: `crates/qmi8658/README.md`
- Driver architecture: `crates/qmi8658/ARCHITECTURE.md`
- Release checklist: `crates/qmi8658/RELEASE_CHECKLIST.md`
- Project policies: `SECURITY.md`, `CHANGELOG.md`, `CODE_OF_CONDUCT.md`

If you change driver behavior or flow sequencing, update both
`crates/qmi8658/README.md` and `crates/qmi8658/ARCHITECTURE.md`.

## Code Style
- Follow existing style and naming conventions.
- Keep changes focused and include tests where possible.

## Pull Request Checklist
- Build passes for affected components.
- Changes are documented (README or CHANGELOG if applicable).
- Hardware-impacting changes are called out in the PR description.

## License
By contributing, you agree that your contributions will be licensed under the MIT License.
