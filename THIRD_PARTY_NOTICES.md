# Third-party notices

## musl libm (partial port)

`core/src/math/detmath/` is a partial C++ port of the double-precision
transcendental functions (`sin`, `cos`, `sincos`, `tan`, `asin`, `acos`,
`atan`, `atan2`, `exp`, `log`, `pow`, and their shared argument-reduction and
polynomial-kernel helpers) from **musl libc's math library**, used as
`ps::math` (see `core/include/ps/math/transcendental.h`). This exists so the
core never depends on the platform C library's transcendental functions,
which round differently between Windows (ucrtbase) and Linux (glibc) and
therefore break cross-platform determinism (PLAN.md D7; vault ticket
PHYS-009).

- Upstream: <https://git.musl-libc.org/cgit/musl/>
- Version: **musl 1.2.5**, tag `v1.2.5`, commit
  `0784374d561435f7c787a555aeab8ede699ed298`.
- Files ported (musl paths, under `src/math/`): `__rem_pio2.c`,
  `__rem_pio2_large.c`, `__sin.c`, `__cos.c`, `__tan.c`, `sin.c`, `cos.c`,
  `tan.c`, `sincos.c`, `asin.c`, `acos.c`, `atan.c`, `atan2.c`, `exp.c`,
  `exp_data.c`, `log.c`, `log_data.c`, `pow.c`, `pow_data.c`, plus the small
  bit-level/rounding helper macros from `src/internal/libm.h`. Only the
  `double`/`double_t` code paths are ported (this project never uses `float`
  or `long double` transcendentals); the `__FP_FAST_FMA` branches of
  `exp.c`/`log.c`/`pow.c` are not ported — the port always takes upstream's
  no-FMA path, since FMA contraction is disabled project-wide (PLAN.md D7)
  and the port never calls a compiler FMA builtin.
- Each ported `.cpp`/`.h` file in `core/src/math/detmath/` carries musl's own
  file-level copyright header, reproduced verbatim from the upstream source.
- Licence: MIT (below), except that the trigonometric and inverse-trig files
  additionally carry a Sun Microsystems/SunSoft/SunPro fdlibm-derived notice
  ("Permission to use, copy, modify, and distribute this software is freely
  granted, provided that this notice is preserved"), also reproduced
  verbatim in those files, per musl's own COPYRIGHT file: "Much of the math
  library code (src/math/* and src/complex/*) is Copyright (C) 1993,2004 Sun
  Microsystems or ... or Copyright (C) 2017-2018 Arm Limited and labelled as
  such in comments in the individual source files. All have been licensed
  under extremely permissive terms."

### musl licence text (from musl's `COPYRIGHT` file at the above commit)

```
musl as a whole is licensed under the following standard MIT license:

----------------------------------------------------------------------
Copyright (C) 2005-2020 Rich Felker, et al.

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
----------------------------------------------------------------------
```

The individual Sun/SunSoft/SunPro/Arm notices reproduced in each ported file
under `core/src/math/detmath/` are part of, and consistent with, this same
overall MIT licensing (see musl's `COPYRIGHT` file, quoted above, and the
per-directory notice in `core/src/math/detmath/common.h`).

## JoltPhysics, Catch2, nlohmann/json

Fetched via CMake `FetchContent` (see `CMakeLists.txt`), not vendored into
this repository; each carries its own licence (JoltPhysics: MIT; Catch2:
BSL-1.0; nlohmann/json: MIT) at its own upstream repository. No source from
these projects is copied into this file tree outside of the build's
`_deps/` cache.
