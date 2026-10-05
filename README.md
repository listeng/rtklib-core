# rtklib-core

RTKLIB [demo5](https://github.com/rtklibexplorer/RTKLIB) b34L with local patches, plus
platform-neutral PPK helpers. No JNI or other platform code lives here; apps add their own
binding on top.

- `rtklib/` — demo5 library sources and receiver decoders (`rcv/`), with local patches.
  `git log -- rtklib` lists every change made to upstream code.
- `ppk/` — `ppk_core.h` (C API: Unicore/RTCM3 to RINEX, `postpos()` wrapper), GPT3/VMF3.
- `CMakeLists.txt` — builds the static library `rtklib_core`.

## Use

Android (alba-nav) references this checkout by relative path, like gdal4android:

```cmake
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../../../rtklib-core rtklib-core)
target_link_libraries(<target> rtklib_core)
```

Desktop build check:

```bash
cmake -S . -B build && cmake --build build -j
```

RTKLIB keeps its state in globals: never run two `ppk_*` operations concurrently.

## License

RTKLIB is BSD-2-Clause (see the header of `rtklib/rtklib.h`).
