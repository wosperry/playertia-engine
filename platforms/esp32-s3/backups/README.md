# Local Factory Backup

The factory firmware image is intentionally excluded from Git by the
repository-wide `*.bin` rule.

Backup captured before the first Playertia flash:

| Field | Value |
| --- | --- |
| Board | Waveshare ESP32-S3-Touch-AMOLED-1.8 V2 |
| Date | 2026-07-29 |
| File | `factory-v2-2026-07-29.bin` |
| Offset / length | `0x0` / `0x1000000` |
| Size | 16,777,216 bytes |
| SHA-256 | `eb47ab438b267bb13fabe8bae9c48e88732ec1bb1a1aa571e13724ae48a3e039` |
| Tool | Espressif esptool.py v4.12.0 from `espressif/idf:v5.5.5` |

Verify:

```bash
sha256sum factory-v2-2026-07-29.bin
```

Restore only when the exact file and hash match:

```bash
esptool.py --chip esp32s3 \
  --port /dev/ttyACM0 \
  --baud 921600 \
  write_flash 0x0 factory-v2-2026-07-29.bin
```

The backup can contain device-specific data and must not be published.
