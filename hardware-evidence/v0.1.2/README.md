# v0.1.2 Hardware Evidence

Place the generated BE-01 `.log`, `.json`, and `.md` run artifacts in this
directory.

The directory is intentionally incomplete until real QMI8658A and QMI8658C
hardware is exercised. Do not add fabricated or software-only evidence.

Follow [`apps/qa-runner/README.md`](../../apps/qa-runner/README.md) to collect
the eight required `variant × transport × byte-order` rows and generate
`BE-01-MATRIX.json` plus `BE-01-MATRIX.md`.

Publication remains blocked until the generated matrix report says `PASS`, the
raw evidence is reviewed, and `V0.1.2_REMEDIATION_PLAN.md` links the committed
artifacts.
