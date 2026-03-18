# Third-party dependencies

This directory stores prebuilt ONNX Runtime binaries in a deduplicated layout:

- `onnxruntime/include`: shared headers (common for all architectures)
- `onnxruntime/pkgconfig`: shared pkg-config file
- `onnxruntime/x86_64`: x86_64 libraries
- `onnxruntime/aarch64`: ARM64 libraries

Top-level metadata files (`LICENSE`, `README.md`, `VERSION_NUMBER`, etc.) are also shared once under `onnxruntime/`.

`../CMakeLists.txt` selects the package automatically based on `CMAKE_SYSTEM_PROCESSOR`:

- `aarch64` or `arm64` -> `onnxruntime/aarch64`
- others -> `onnxruntime/x86_64`

The build script always uses shared headers from `onnxruntime/include` and auto-detects the shared library path from either `lib/` or `lib64/`.
