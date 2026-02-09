# Publishable Roadmap (Temporary)

Goal: move `ph-qmi8658` from internal development to a publishable, documented, and tested crate.
This plan is organized into short sprints with explicit deliverables. Update as work lands.

## Current Sprint
- Sprint 2: Tests & CI

## Current Snapshot (Codebase Observations)
- Driver crate lives in `crates/qmi8658` as `ph-qmi8658`.
- Apps exist for ESP32-S3 (`apps/qa-runner`, `apps/imu-example`) and are excluded from the workspace.
- Public docs exist at repo root and driver root; architecture moved to driver root.
- A helper init macro exists: `qmi8658_init_sequence!`.
- Docs/examples now reference `ph_qmi8658`.
- Publishability blockers: audit of public API docs, tests, and release process.

## Sprint 0 (1-2 days): Metadata & Hygiene
Status: complete.
Deliverables:
- Audit and fix crate metadata in `crates/qmi8658/Cargo.toml`:
  - `license`, `repository`, `readme`, `keywords`, `categories`.
  - Set `publish = true` if not already.
- Verify all docs/examples use `ph_qmi8658` consistently.
- Ensure `README.md` at repo root links to driver docs and architecture.
- Run `cargo fmt` + `cargo clippy` on driver crate.

Acceptance:
- `cargo package -p ph-qmi8658` succeeds.

## Sprint 1 (3-5 days): API Surface & Docs
Status: complete.
Deliverables:
- Review public API for stability: rename or deprecate anything that is too verbose or unclear.
- Add doc comments for all public items not already documented.
- Add targeted examples for the core flows in `crates/qmi8658/README.md`:
  - I2C init + config
  - FIFO burst read
  - Sync sample flow
  - WoM flow
- Ensure `ARCHITECTURE.md` and README remain aligned with behavior.

Acceptance:
- `cargo doc -p ph-qmi8658` builds without warnings.

Progress:
- Public API review: no changes required at this time.
- README examples cover all core flows.
- `ARCHITECTURE.md` aligned with current module map and init macro.
- `cargo doc -p ph-qmi8658` succeeds without warnings.

## Sprint 2 (3-5 days): Tests & CI
Status: in progress.
Deliverables:
- Unit tests for config validation, FIFO parsing, and interrupt decoding.
- Mock-based integration tests for init/config flows using the existing mock interface.
- Add CI to run `cargo test -p ph-qmi8658` and `cargo clippy`.
- Add `cargo deny` (licenses/advisories) if you plan to publish on crates.io.

Acceptance:
- All tests green in CI.

## Sprint 3 (2-4 days): Publish Readiness
Status: pending.
Deliverables:
- Final pass on README, CHANGELOG, and security policy.
- Create a release checklist (tagging, version bumping, crate publish).
- Publish `v0.1.0` and start tracking changes in `CHANGELOG.md` afterward.
 - Document supported CLI targets:
  - All ESP32 variants (xtensa + riscv32).
  - Common ARM Cortex-M targets.

Acceptance:
- `cargo publish -p ph-qmi8658 --dry-run` succeeds.

Progress:
- Documented target matrix in `crates/qmi8658/README.md`.
- Added `crates/qmi8658/RELEASE_CHECKLIST.md`.

Sprint 2 Progress:
- Added CI workflow with target matrix builds.
- Added `deny.toml` for cargo-deny.

## Optional Sprint 4: Post-0.1 Hardening
Deliverables:
- Add SPI parity and sample apps.
- Performance profiling on ESP32-S3 and note limits.
- Consider feature gating experimental flows.

---
Temporary doc. Delete or replace once you formalize the release plan.
