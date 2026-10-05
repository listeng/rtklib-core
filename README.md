# rtklib-core

RTKLIB [demo5](https://github.com/rtklibexplorer/RTKLIB) b34L with local patches, plus
platform-neutral PPK helpers. No JNI or other platform code lives here; apps add their own
binding on top.

- `rtklib/` — demo5 library sources and receiver decoders (`rcv/`), with local patches,
  plus local extensions the patched code calls directly (`gpt3.c/.h`, `vmf3_coef.h`: GPT3
  grid and VMF3 mapping functions). `git log -- rtklib` lists every change to the core.
- `api/` — `ppk_core.h`, the C API apps call (Unicore/RTCM3 to RINEX, `postpos()` wrapper).
  It depends on `rtklib/`, never the other way round.
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
