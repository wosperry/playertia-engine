# Upstream Source Record

This directory is an unmodified, size-limited reference snapshot of the
official Waveshare repository:

- Repository: <https://github.com/waveshareteam/ESP32-S3-Touch-AMOLED-1.8>
- Commit: `8badafee5633d3786db00aa64dcf1977d5df6707`
- Commit date: 2026-07-24
- Upstream tag at the pinned commit: `v1.0.1`
- Upstream license: Apache License 2.0; see [LICENSE](LICENSE)

Included paths:

- `LICENSE`, `THIRD_PARTY.md`, upstream readmes;
- `config/` and `docs/`;
- `examples/esp-idf/`.

Intentionally excluded:

- `Firmware/` recovery binaries;
- Arduino example trees and bundled library copies;
- release artifacts, images, CI metadata, and repository history.

The upstream `Firmware/` directory contains factory/recovery **binaries only**.
Waveshare's own `docs/FIRMWARE.md` states that the corresponding factory source
and build instructions are not included in the official repository yet.
Therefore this snapshot cannot truthfully include the source of the exact
factory Xiaozhi firmware. The included ESP-IDF projects are the official,
source-buildable reference implementations.

The full upstream worktree is roughly 599 MB at this revision, while the
ESP-IDF reference source used by this project is about 1 MB. Excluding the
large paths prevents third-party binary and duplicated library payloads from
becoming part of Playertia's source history. Retrieve the complete upstream
revision when a factory image or Arduino-specific reference is needed:

```bash
git clone https://github.com/waveshareteam/ESP32-S3-Touch-AMOLED-1.8.git
cd ESP32-S3-Touch-AMOLED-1.8
git checkout 8badafee5633d3786db00aa64dcf1977d5df6707
```

Do not edit upstream files in place. Put Playertia-specific platform code
under `platforms/esp32-s3/`.
