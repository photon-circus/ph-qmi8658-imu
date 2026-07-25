# QMI8658 Hardware Evidence Runner

This ESP32-S3 firmware and host-side collector produce the evidence required to
resolve `BE-01` in `V0.1.2_REMEDIATION_PLAN.md`.

The workflow covers the complete matrix:

| Variant | Transport | CTRL1.BE |
|---|---|---|
| QMI8658A | I2C | 0 and 1 |
| QMI8658A | SPI | 0 and 1 |
| QMI8658C | I2C | 0 and 1 |
| QMI8658C | SPI | 0 and 1 |

Each run captures:

- firmware variant, transport, and requested byte order;
- host source commit, dirty-tree state, operator, board, and sensor identifiers;
- WHO_AM_I, REVISION_ID, and CTRL1 readback;
- 16 direct register samples as exact bytes and decoded values;
- FIFO metadata, exact frame bytes, and decoded values;
- static +Z 1 g, stationary gyro, temperature, direct/FIFO, byte-order, and
  transport consistency checks.

A successful individual run does not clear the release blocker. The complete
eight-row matrix must pass and be reviewed.

## Prerequisites

Install:

- Python 3.10 or later;
- the Espressif Rust `esp` toolchain;
- `espflash` 4.x;
- an ESP32-S3 development board;
- one identified QMI8658A and one identified QMI8658C sensor.

Run collection from a clean checkout of the exact candidate commit. Use the
same physical A sensor for all four A rows and the same physical C sensor for
all four C rows. Give each sensor a stable label such as `A-BOARD-01`.

The collector builds firmware, flashes the ESP32-S3, monitors serial output,
stops at `QMI_EVIDENCE_END`, validates the run, and writes `.log`, `.json`, and
`.md` artifacts.

## Electrical Safety and Sensor Placement

For a bare QMI8658:

- VDD and VDDIO must remain within the vendor-recommended 1.71-3.6 V range.
- Never connect the sensor to 5 V.
- Connect ESP32-S3 ground and sensor ground.
- Add the vendor-recommended 100 nF decoupling capacitors at VDD and VDDIO.
- Power down before changing between I2C and SPI wiring.

A breakout board may contain regulators, level shifting, pull-ups, or fixed
interface straps. Follow its schematic and record the board model in
`--sensor-id` or `--notes`.

Place the sensor motionless on a rigid, level surface with the sensor package
**+Z axis pointing upward**. Use the package coordinate diagram from the
appropriate vendor datasheet; do not assume the breakout board artwork uses
the same orientation.

## ESP32-S3 Wiring

The firmware uses these fixed GPIO assignments.

### I2C

| ESP32-S3 | QMI8658 signal | Bare-device pin | Notes |
|---|---|---:|---|
| GPIO11 | SDA | 14 | Add a 2-10 kΩ pull-up to VDDIO |
| GPIO12 | SCL | 13 | Add a 2-10 kΩ pull-up to VDDIO |
| GPIO10 | INT1 | 4 | Optional for BE evidence, connected by runner |
| GPIO13 | INT2 | 9 | Optional for BE evidence, connected by runner |
| VDDIO | CS | 12 | Tie high to select I2C/I3C operation |
| VDDIO or float | SDO/SA0 | 1 | Address 0x6A |
| GND | SDO/SA0 | 1 | Alternative address 0x6B |

The firmware probes both 0x6A and 0x6B at 400 kHz.

### Four-wire SPI

| ESP32-S3 | QMI8658 signal | Bare-device pin |
|---|---|---:|
| GPIO11 (MOSI) | SDI | 14 |
| GPIO14 (MISO) | SDO | 1 |
| GPIO12 (SCLK) | SPC | 13 |
| GPIO9 | CS | 12 |
| GPIO10 | INT1 | 4 |
| GPIO13 | INT2 | 9 |

The runner uses SPI Mode 0, MSB first, at 1 MHz. Both referenced vendor
datasheets permit Mode 0 or Mode 3 and specify a maximum 15 MHz SPI clock.

## Collect One Sensor

Record the ambient temperature before starting. From the repository root, run
the two byte-order settings after wiring the selected transport. Replace the
port and identifiers with your setup.

PowerShell:

```powershell
$variant = "qmi8658a"
$transport = "i2c"
$port = "COM7"
$operator = "initials"
$board = "ESP32S3-QA-01"
$sensor = "A-BOARD-01"
$ambient = 23.5

foreach ($order in @("le", "be")) {
  python apps/qa-runner/tools/collect_evidence.py collect `
    --variant $variant `
    --transport $transport `
    --byte-order $order `
    --port $port `
    --operator $operator `
    --board-id $board `
    --sensor-id $sensor `
    --ambient-celsius $ambient
  if ($LASTEXITCODE -ne 0) {
    Write-Warning "Run failed validation; preserve its artifacts and investigate."
  }
}
```

Bash:

```bash
variant=qmi8658a
transport=i2c
port=/dev/ttyACM0
operator=initials
board=ESP32S3-QA-01
sensor=A-BOARD-01
ambient=23.5

for order in le be; do
  python3 apps/qa-runner/tools/collect_evidence.py collect \
    --variant "$variant" \
    --transport "$transport" \
    --byte-order "$order" \
    --port "$port" \
    --operator "$operator" \
    --board-id "$board" \
    --sensor-id "$sensor" \
    --ambient-celsius "$ambient" || true
done
```

Then:

1. Power down and rewire the same sensor for the other transport.
2. Change `transport` to `spi` and repeat LE/BE.
3. Power down, install the other sensor variant, change `variant` and
   `sensor`, and repeat both transports and byte orders.
4. Do not delete failed artifacts. Initialization failure, timeout, implausible
   data, or byte-order inconsistency is evidence and keeps BE-01 blocked.

If flashing and monitoring must be performed separately, save the complete
serial output and pass it through the same validator:

```powershell
python apps/qa-runner/tools/collect_evidence.py collect `
  --variant qmi8658c --transport spi --byte-order le `
  --operator initials --board-id ESP32S3-QA-01 --sensor-id C-BOARD-01 `
  --ambient-celsius 23.5 --input-log captured-serial.log
```

## Validate the Complete Matrix

After all eight runs:

```powershell
$candidateCommit = git rev-parse HEAD
python apps/qa-runner/tools/collect_evidence.py matrix `
  --input-dir hardware-evidence/v0.1.2 `
  --expected-source-commit $candidateCommit
```

The command selects the newest run for each matrix row and generates:

- `hardware-evidence/v0.1.2/BE-01-MATRIX.json`
- `hardware-evidence/v0.1.2/BE-01-MATRIX.md`

It exits successfully only when:

- all eight rows exist and passed their individual checks;
- every run came from a clean tree at the expected source commit;
- each variant uses one consistent sensor ID and revision ID;
- decoded direct and FIFO values remain consistent between BE=0 and BE=1;
- decoded values remain consistent between I2C and SPI.

Review the raw logs and JSON, confirm `source_dirty` is `false`, and confirm the
reported sensor markings/revision IDs match the physical hardware. Commit the
evidence artifacts and update `BE-01` plus the release-gate links in
`V0.1.2_REMEDIATION_PLAN.md` in the same commit. Do not mark BE-01 `Done` from
an individual-run report.

## Offline Tool Tests

The collector uses only the Python standard library. Test its parser,
raw-byte cross-check, and matrix gating with:

```powershell
python -m unittest discover -s apps/qa-runner/tools -p "test_*.py" -v
```

## Datasheet Basis

The wiring and protocol settings above were checked against:

- `docs/vendor/QMI8658A-Datasheet-Rev-D.pdf`, especially pages 7-9 and 68-75;
- `docs/vendor/QMI8658C-Datasheet-Rev-A.pdf`, especially pages 8-10 and 76-84.

Those datasheets define the SDO/SA0, CS, SCL/SPC, and SDA/SDI mapping, I2C
address selection, 2-10 kΩ I2C pull-ups, 4-wire SPI connections, Mode 0/3
operation, 15 MHz SPI limit, and CTRL1.BE behavior.
