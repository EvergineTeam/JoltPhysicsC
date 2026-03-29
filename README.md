# JoltPhysicsC

C bindings for [Jolt Physics](https://github.com/jrouwe/JoltPhysics) — a fast, modern 3D physics engine.

JoltPhysicsC wraps the Jolt Physics C++ API into a flat C API exposed as a shared library (`JoltC.dll`, `libJoltC.so`, `libJoltC.dylib`). It is primarily designed to be consumed by [JoltPhysics.NET](https://github.com/EvergineTeam/JoltPhysics.NET), the C# binding used in [Evergine](https://evergine.com).

## Supported Platforms

| Platform | Architecture | Artifact |
|---|---|---|
| Windows | x64, ARM64 | `JoltC.dll` |
| Linux | x64, ARM64 | `libJoltC.so` |
| macOS | ARM64 | `libJoltC.dylib` |

## Building

### Prerequisites

- [CMake](https://cmake.org/) 3.20 or later
- A C++17 compiler (MSVC, GCC, Clang, or Apple Clang)

### Clone

```bash
git clone --recursive https://github.com/EvergineTeam/JoltPhysicsC.git
cd JoltPhysicsC
```

If you already cloned without `--recursive`, fetch the submodule:

```bash
git submodule update --init --recursive
```

### Configure & Build

**Windows (Visual Studio)**
```bash
cmake -S JoltC -B JoltC/build -A x64
cmake --build JoltC/build --config Release
```

**Linux**
```bash
cmake -S JoltC -B JoltC/build -DCMAKE_BUILD_TYPE=Release
cmake --build JoltC/build
```

**macOS**
```bash
cmake -S JoltC -B JoltC/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_OSX_ARCHITECTURES=arm64
cmake --build JoltC/build
```

### CMake Options

| Option | Default | Description |
|---|---|---|
| `JOLTC_DOUBLE_PRECISION` | `OFF` | Use double precision for world-space positions |
| `PHYSICS_REPO_ROOT` | `../JoltPhysics` | Path to the Jolt Physics source tree |

## API Overview

Include the umbrella header to access the full API:

```c
#include <JoltC/joltc.h>
```

The API is organized into the following modules:

| Header | Description |
|---|---|
| `common.h` | Initialization, shutdown, error handling, opaque handles |
| `math.h` | Vectors, quaternions, matrices |
| `shape.h` | Collision shapes (box, sphere, capsule, mesh, compound, etc.) |
| `physics_system.h` | Physics world creation and simulation stepping |
| `body.h` | Rigid body creation and manipulation |
| `body_access.h` | Scoped body read/write locking |
| `constraint.h` | Joints and constraints |
| `query.h` | Ray casts and shape casts |
| `character.h` | Character controllers |
| `filters.h` | Broadphase and object layer filtering |
| `vehicle.h` | Vehicle simulation |
| `skeleton.h` | Skeleton and animation support |

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.
