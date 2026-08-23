# JoltPhysicsC wrapper profile

What is specific to this repository. The shared half is
[`docs/cpp-wrapper-conventions.md`](https://github.com/EvergineTeam/Evergine.Bindings/blob/main/docs/cpp-wrapper-conventions.md)
in the toolbox, and `cpp-wrapper-porter` reads both.

That document describes how wrappers here are written. **It does not authorise rewriting
this one.** A bump repairs what upstream broke; anything else belongs in a separate change.

## Identifiers

| | |
|---|---|
| Function naming | `JoltC_<Type>_<Method>`, PascalCase, no domain segment |
| Export macro | `JOLTC_API`, gated on `JOLTC_EXPORTS` |
| Error macros | `TRY_BEGIN` / `TRY_END`, unprefixed |
| Last-error accessors | `JoltC_GetLastError`, `JoltC_ClearLastError` |
| Boolean | `typedef int JoltC_Bool`, with `JOLTC_TRUE` and `JOLTC_FALSE` |
| Converters | **Type-suffixed**: `toJphVec3`, `fromJphRVec3` |

The converters are suffixed rather than overloaded for a specific reason: with single
precision `RVec3` is a typedef of `Vec3`, so overloads on both collapse into a redefinition,
and the build breaks somewhere unrelated to the change that caused it.

## Overloads

**Numeric suffixes**: `_Create`, `_Create2`, `_Create3`. If upstream adds an overload, the
next free number is the answer here.

(CesiumC does the opposite — semantic suffixes like `_create_from_url`. Do not carry a habit
across from one profile to the other.)

## Paths

| | |
|---|---|
| Public headers | `JoltC/include/JoltC/` |
| Implementation | `JoltC/src/` |
| Tests | `JoltC/tests/` |
| Build | `JoltC/CMakeLists.txt` |
| Upstream submodule | `JoltPhysics/` |

## Scope contract

**Exhaustive.** This wrapper aims to cover the whole public C++ surface, with
`amerkoleci/joltc` as the reference for what that means in practice. Around 1,280 exported
functions today.

That said, the porter does not add API. Its job is repair plus a report; the exhaustiveness
target is what a human uses when deciding whether to accept the report.

## Applying a bump

One edit: move the `JoltPhysics/` submodule to the new tag. No dependency manifest, no
baseline, no list of upstream libraries — this repository has no external dependencies and
includes `Jolt/Jolt.cmake` directly.

Two things that follow from including `Jolt.cmake` rather than upstream's `CMakeLists.txt`:

- **Upstream's `option()` declarations never run**, so its defaults do not apply here. The
  four compute backends and the library type are pinned explicitly in `JoltC/CMakeLists.txt`
  for that reason. The set of `JPH_*` variables `Jolt.cmake` reads grew from 3 to 11 between
  5.5.0 and 5.6.0, so **check what a new release added to that set** and pin anything that
  changes what gets built.
- A new release may expect a variable nothing here defines, and CMake reads an undefined
  variable as false rather than complaining.

## Building and testing

```bash
cmake -S JoltC -B JoltC/build -DCMAKE_BUILD_TYPE=Release -DJOLTC_BUILD_TESTS=ON \
  -DPHYSICS_REPO_ROOT="$PWD/JoltPhysics"
cmake --build JoltC/build --config Release
ctest --test-dir JoltC/build --build-config Release --output-on-failure --no-tests=error
```

Eleven suites, hand-rolled C, no framework. Non-zero exit on any failure, and no results
that mean "skipped" — every test either runs or the run failed.

The libraries are built for ten platforms by `.github/workflows/build-joltc.yml`, which runs
on every push and pull request. **Tests run on the three whose binaries the runner can
execute** — `linux-x64`, `win-x64`, `osx-arm64`. The other seven are cross-compiled and only
build, so a green run proves the desktop subset behaves and the rest compiles.

A pull request gets all ten legs automatically. A branch without a pull request gets nothing:
`push` is restricted to `main`.

## Pattern modules that apply here

- **Reference counting exposed.** Shapes are `AddRef`/`Release`; the C caller owns a
  reference it took.
- **Abstract interface subclassing.** Contact listeners and filters are C++ subclasses
  forwarding to a function pointer plus `void* userData`.

Not applicable: completion callbacks that transfer ownership, and structs of function
pointers with an embedded `userData`. Those are CesiumC's shape, because cesium-native is
future-based. Nothing here is.

## Version

`JOLTC_JOLT_VERSION` in `JoltC/include/JoltC/common.h`, plus the `_MAJOR`, `_MINOR` and
`_PATCH` integers, and a line in the README. Keep them in step with `release.current` in
`binding.yml`.

One caution learned the hard way: that macro is a **quoted string**, and the C# generator
downstream classified it as a float because the value contains a dot. It emitted
`public const float JOLT_VERSION = "5.5.0";` and broke the binding's build. The generator is
fixed, but a new string macro is worth a thought.

## Known local quirks

- **`vehicle.cpp` uses token-pasting macros** (`WS_VEC3_PROP`, `MCS_FLOAT` and others) that
  expand to getter and setter pairs. The count of `JOLTC_API` in the sources is therefore
  lower than the count in the headers, and that gap is not missing code.
- The nine `*Constraint_GetSettings` used to be NULL-returning stubs; they are real since the
  phase 0 coverage work (`SETTINGS_GETTER` in `src/constraint.cpp`), returning a counted
  `JoltC_TwoBodyConstraintSettings*` the caller releases.

## Known coverage gaps, phased

An audit against Jolt 5.6 (2026-08) found the wrapper short of its "exhaustive" contract in a
number of places. Phase 0 (enum values matching Jolt's, real GetSettings and the ragdoll
articulation they unlock, CharacterVirtual filters, the soft body contact listener and manifold,
query settings connected to their queries, per-system broad phase query handles) is done.
Phase 1 is done too: physics materials run end to end (`MeshShape_Create2` and
`HeightFieldShape_Create2` take material lists, `Shape_GetMaterial` and the character
`GetGroundMaterial` hand them back, and `JoltC_PhysicsMaterial*` became the same raw
ref-counted cast as `JoltC_Shape*` so identity comparison works), plus shape introspection:
the `GetTrianglesStart/Next` walk, `GetLeafShape`, `GetSubShapeUserData`, `GetSubmergedVolume`,
and height field runtime deformation (`Get/SetHeights`, `Get/SetMaterials` per cell).
Phase 2 completed the soft body surface: `CreateConstraints2` takes per-vertex attributes (which
is what finally creates LRA constraints), every constraint list can be built directly (edges,
dihedral bends, volumes, LRA, Cosserat rods with their bend/twist coupling), skinning runs end to
end (`AddInvBindMatrix`/`AddSkinnedConstraint`/`SkinVertices` and the runtime toggles), vertices
are writable at runtime (position, velocity, inverse mass, plus `CalculateMassAndInertia`), the
creation settings expose `mFacesDoubleSided`, `mCollisionGroup` and getters for everything, and
`CustomUpdate` steps a body that lives outside the system. Still open, in planned order:

1. Determinism: `StateRecorder`, `PhysicsSystem::SaveState/RestoreState` and the per-object
   SaveState family.
2. Constraints and vehicles: `PathConstraint` (its enum value exists and nothing else does),
   Pulley runtime accessors, vehicle step callbacks and the WheelWV slip fields.
3. Character: the eight missing `CharacterContactListener` callbacks with the full
   `CharacterContact` payload, custom character-vs-character procs, `mSupportingVolume`;
   `MotionProperties` completion; collectors with early-out.
4. `DebugRenderer` through C procs.

Deliberately out of scope unless asked for: hair simulation, the compute shader interface, and
`ObjectStream` serialization.
