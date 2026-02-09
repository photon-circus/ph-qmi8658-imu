# Release Checklist (ph-qmi8658)

## Preflight
- Confirm `version` and `rust-version` in `crates/qmi8658/Cargo.toml`.
- Verify `README.md`, `ARCHITECTURE.md`, `SECURITY.md`, and `CHANGELOG.md` are up to date.
- Ensure docs/examples use `ph_qmi8658`.

## Build & Test
- `cargo fmt -p ph-qmi8658`
- `cargo clippy -p ph-qmi8658`
- `cargo doc -p ph-qmi8658`
- `cargo test -p ph-qmi8658`

## Target Matrix (CLI Builds)
ESP32 (xtensa):
- `xtensa-esp32-none-elf`
- `xtensa-esp32s2-none-elf`
- `xtensa-esp32s3-none-elf`

ESP32 (riscv32):
- `riscv32imc-unknown-none-elf`
- `riscv32imac-unknown-none-elf`

ARM Cortex-M (common):
- `thumbv6m-none-eabi`
- `thumbv7m-none-eabi`
- `thumbv7em-none-eabi`
- `thumbv7em-none-eabihf`
- `thumbv8m.base-none-eabi`
- `thumbv8m.main-none-eabi`
- `thumbv8m.main-none-eabihf`

## Packaging
- `cargo package -p ph-qmi8658 --allow-dirty`
- `cargo publish -p ph-qmi8658 --dry-run`

## Release
- Tag the version in git.
- Publish to crates.io.
- Start tracking changes in `CHANGELOG.md` after v0.1.0.
