# OpenApex - Simracing Game Development Plan

> **Vision**: A fully open-source, community-driven simracing platform with iRacing-level physics, modern Vulkan graphics, good netcode, and unlimited modding.
>
> **Current Status**: Phase 1 (core physics) and Phase 2 (ABS/TC/LSD/UI) complete. Phase 3 (Vulkan renderer structure) implemented but not yet rendering.

---

## 📊 Phase Overview

| Phase | Focus | Status | Target |
|-------|-------|--------|--------|
| **0** | Foundation & Tooling | 🔄 In Progress | Week 1 |
| **1** | First Drivable Demo | ⏳ Pending | Week 2-3 |
| **2** | Physics Deepening | ⏳ Pending | Month 2-3 |
| **3** | Content Pipeline & Modding | ⏳ Pending | Month 3-4 |
| **4** | Multiplayer & Netcode | ⏳ Pending | Month 4-6 |
| **5** | Damage & Advanced Systems | ⏳ Pending | Month 5-7 |
| **6** | Modern Graphics | ⏳ Pending | Month 6+ |
| **7** | Community & Governance | ⏳ Pending | Ongoing |

---

## ✅ Phase 0: Foundation & Tooling (Week 1)

### Build & CI
- [x] CMake + vcpkg working for all targets
- [x] sim-core library with 46 tests passing
- [x] Vulkan renderer structure compiles
- [ ] **GitHub Actions CI**: Linux/Windows/macOS build matrix
- [ ] **Physics regression test**: "Car X on Track Y = laptime Z ± 0.1s" in CI
- [ ] **Shader validation**: spirv-val in CI
- [ ] **Static analysis**: clang-tidy + cppcheck in CI
- [ ] **Release artifacts**: AppImage, Windows installer, macOS .app

### Licensing & Governance
- [ ] Add REUSE-compliant license headers to all files
- [ ] Choose licenses:
  - [ ] sim-core, renderer, app → **GPLv3**
  - [ ] net, audio, ffb, tools, mods API → **MIT/Apache-2.0**
  - [ ] content (cars/tracks/sounds) → **CC-BY-SA 4.0**
  - [ ] Brand (name/logo) → **Trademark**
- [ ] Add `DCO` (Developer Certificate of Origin) - Signed-off-by commits
- [ ] Create `CONTRIBUTING.md`, `GOVERNANCE.md`, `CODE_OF_CONDUCT.md`
- [ ] Set up GitHub Discussions categories

### Repository Structure
```
openapex/
├── sim-core/           # GPLv3 - physics, vehicle, track logic
├── renderer-vulkan/    # GPLv3 - rendering only
├── audio/              # MIT - miniaudio + granular synthesis
├── net/                # MIT - GNS wrapper, replication
├── ffb/                # MIT - force feedback pipeline
├── app/                # GPLv3 - main executable, SDL3 integration
├── tools/              # MIT - Blender exporters, validators, track editor
├── content/            # CC-BY-SA - cars, tracks, sounds
├── mods/               # MIT - plugin API (Lua/WASM)
├── .github/workflows/  # CI pipelines
└── LICENSES/           # Per-file license mapping
```

---

## 🚗 Phase 1: First Drivable Demo (Week 2-3)

> **Goal**: One car, one track, working FFB, ugly but drivable. Physics feel > graphics.

### Vulkan Renderer Integration
- [ ] **Compile shaders to SPIR-V**
  ```bash
  glslangValidator -V renderer/vulkan/shaders/track.vert -o track.vert.spv
  glslangValidator -V renderer/vulkan/shaders/track.frag -o track.frag.spv
  glslangValidator -V renderer/vulkan/shaders/car.vert -o car.vert.spv
  glslangValidator -V renderer/vulkan/shaders/car.frag -o car.frag.spv
  glslangValidator -V renderer/vulkan/shaders/ui.vert -o ui.vert.spv
  glslangValidator -V renderer/vulkan/shaders/ui.frag -o ui.frag.spv
  ```
- [ ] **Hook renderer into `app/main.cpp`** - replace SDL software rendering
- [ ] **Verify track loads** - camera follows car, correct position/orientation
- [ ] **Frame timing** - fixed physics timestep (360Hz), interpolated render
- [ ] **Input handling** - SDL3 keyboard + wheel support

### Debug Visualization (ImGui)
- [ ] Tire forces (Fx, Fy, Fz, slip angles, slip ratios)
- [ ] Suspension travel, spring/damper forces
- [ ] Aero forces, downforce/drag
- [ ] Vehicle state: speed, RPM, gear, G-forces
- [ ] Track surface query at contact patches

### Force Feedback (Separate Thread)
- [ ] 1kHz FFB thread architecture
- [ ] Rack force calculation from tire alignment torque
- [ ] Basic effects: weight transfer, curb rumble, slip vibration
- [ ] SDL3 haptics + direct USB HID path

### Content: First Car & Track
- [ ] **Define `car.toml` schema** (see Phase 3)
- [ ] **Define `track.json` extensions** (surface, objects, pits)
- [ ] Create placeholder car: simple glTF box + 4 wheels
- [ ] Create test track: flat oval with kerbs (extend demo.track.json)
- [ ] Validate physics: steady-state cornering, brake/accel tests

---

## ⚙️ Phase 2: Physics Deepening (Month 2-3)

### Tire Model Evolution
- [ ] **Pacejka 2002 baseline** (current) ✅
- [ ] **Temperature model**: core/surface temp, heat generation/conduction
- [ ] **Pressure model**: pressure ↔ temp ↔ load sensitivity
- [ ] **Wear model**: tread depth → grip loss, puncture risk
- [ ] **Relaxation length**: transient slip response (critical for FFB feel)
- [ ] **Combined slip**: proper Fx/Fy coupling (current: simplified)
- [ ] **Validation**: Compare skidpad, sweep tests vs real data

### Suspension Geometry
- [ ] **Double wishbone kinematics**: K&C (kinematics & compliance)
- [ ] **Pushrod/pullrod** motion ratios
- [ ] **Non-linear dampers**: velocity-dependent, bump/rebound separate
- [ ] **Bump stops**: progressive rate, damper blow-off
- [ ] **Anti-roll bars**: blade vs tube, adjustable
- [ ] **Steering**: rack ratio, Ackermann, compliance

### Drivetrain
- [ ] **Engine**: torque curve, inertia, rev limiter, throttle response
- [ ] **Clutch**: slip model, wear, thermal fade
- [ ] **Gearbox**: ratios, shift times, synchro model, sequential/H-pattern
- [ ] **Differentials**: 
  - [ ] Open (done)
  - [ ] Viscous (done)
  - [ ] Clutch-type LSD: preload, ramp angles, plate count (done - needs tuning)
  - [ ] Torsen: bias ratio, preload
- [ ] **Driveshafts**: inertia, compliance

### Aerodynamics
- [ ] **Ride-height/pitch dependent** CL/CD tables (CSV/interpolation)
- [ ] **Front/rear aero balance** → handling tuning
- [ ] **DRS** (if applicable)
- [ ] **Draft/wake model**: reduced drag/downforce in slipstream
- [ ] **Ground effect** (for relevant categories)

### Track Surface
- [ ] **OpenCRG-style** height + grip map per segment
- [ ] **Surface types**: asphalt variants, kerbs, grass, gravel, astroturf
- [ ] **Dynamic rubbering-in**: grip evolution during session
- [ ] **Wet weather**: water film depth, aquaplaning model

---

## 📦 Phase 3: Content Pipeline & Modding (Month 3-4)

### Data Formats (Human-Readable, Schema-Validated)
- [ ] **`car.toml` schema** - Vehicle definition
  ```toml
  [meta]
  name = "Generic GT3"
  author = "OpenApex"
  version = 1
  
  [chassis]
  mass = 1300.0
  cog = [0.0, 0.3, 0.0]
  inertia = [1200.0, 1800.0, 2500.0]
  
  [aero]
  cl_curve = "data/aero/cl_vs_rideheight.csv"
  cd_curve = "data/aero/cd_vs_rideheight.csv"
  
  [suspension.fl]
  # Double wishbone hardpoints, spring, damper, ARB, bumpstop
  ...
  
  [tire]
  pacejka = "data/tires/gt3_slick.pacejka"
  ...
  
  [drivetrain]
  engine = "data/engines/gt3_v8.toml"
  gearbox = "data/gearboxes/gt3_6spd.toml"
  diff = "data/diffs/gt3_clutch.toml"
  ```
- [ ] **`track.json` extensions** - Surface, objects, AI lines, pits
- [ ] **JSON Schema** for validation in CI

### Blender Exporter (Python)
- [ ] Export track mesh (GLTF) + collision mesh + surface map
- [ ] Export car model (GLTF) + wheel bones + damage zones
- [ ] Export collision primitives (convex hulls for tires, chassis)
- [ ] **Validation tool**: `openapex-validate track.json` / `car.toml`

### Modding API
- [ ] **Lua scripting** (sol2) for:
  - HUDs / dash apps
  - Race logic (flags, penalties, pit rules)
  - Telemetry export
- [ ] **WASM sandbox** (wasmtime) for untrusted mods
- [ ] **Plugin interface**: C API for native mods
- [ ] **Mod loader**: discovery, dependency resolution, versioning

### Content Creation
- [ ] 2-3 fictional tracks (fantasy + real-world inspired)
- [ ] 3-4 cars across categories (GT3, Formula, Touring, Street)
- [ ] Community car/track pack template repo

---

## 🌐 Phase 4: Multiplayer & Netcode (Month 4-6)

### Architecture
```
┌─────────────────┐     20-60 Hz snapshots      ┌─────────────────┐
│    Client       │ ◄───────────────────────── │  Dedicated      │
│  (prediction)   │   delta-compressed,        │  Server         │
│                 │   quantized, encrypted     │  (authoritative)│
└────────┬────────┘                            └────────┬────────┘
         │                                              │
         ▼                                              ▼
   Local car @ 360Hz                            All cars @ 360Hz
   Reconciliation                                Collision resolution
   Remote cars: dead reckoning                   Lag compensation
```

### Implementation
- [ ] **GameNetworkingSockets** integration (Valve, BSD license)
- [ ] **Server**: headless sim-core, config-driven
- [ ] **Client prediction**: local car @ 360Hz, reconcile on snapshot
- [ ] **Remote cars**: dead reckoning + cubic spline interpolation
- [ ] **Snapshots**: 20-60 Hz, delta compression, quantization
- [ ] **Lag compensation**: rewind/replay for collision resolution
- [ ] **Join-in-progress**: state transfer, warmup sync
- [ ] **Spectator / replay** system

### Security & Anti-Cheat (Basic)
- [ ] Input validation on server
- [ ] Physics determinism checks
- [ ] Replay-based verification

---

## 💥 Phase 5: Damage & Advanced Systems (Month 5-7)

### Parametric Damage (iRacing-style)
- [ ] **Damage state per component**:
  - Suspension: camber/toe shift, spring rate loss, damper fade
  - Aero: downforce/drag reduction, CoP shift
  - Engine: torque degradation, rev limiter drop, oil pressure
  - Tires: puncture → pressure loss → Pacejka degradation
  - Drivetrain: diff lock loss, gear engagement issues
- [ ] **Visual deformation**: lattice/FFD per zone on body mesh
- [ ] **Repair model**: pit stop time per damage type

### Audio (miniaudio + Granular Synthesis)
- [ ] **Engine**: granular synthesis from loop + load/RPM layers
- [ ] **Transmission**: whine, gear clunk, shift sounds
- [ ] **Tires**: scrub, squeal, rumble (surface-dependent)
- [ ] **Aero**: wind noise, DRS flap
- [ ] **Ambient**: crowd, PA, weather
- [ ] **Occlusion/Reverb**: simple room model

### Telemetry & Analysis
- [ ] **Live telemetry** (WebSocket → browser overlay)
- [ ] **Replay system**: deterministic playback, camera tools
- [ ] **Data export**: CSV, MoTeC LD, iRacing IBT compatible

---

## 🎨 Phase 6: Modern Graphics (Month 6+)

| Milestone | Techniques |
|-----------|------------|
| **v0.2** | Clustered forward+, cascaded shadow maps, GTAO |
| **v0.3** | TAA, bloom, ACES tonemapping, rain/wet track shader |
| **v0.4** | Bindless, GPU culling, mesh shaders (VK_EXT_mesh_shader) |
| **v0.5+** | Optional RT: shadows, reflections (separate pipeline) |

### Vulkan Renderer Checklist
- [ ] **Forward+ / clustered** lighting
- [ ] **PBR** with IBL (prefiltered env maps, BRDF LUT)
- [ ] **CSM** for sun shadows (4 cascades, PCF/PCSS)
- [ ] **Screen-space**: GTAO, SSR (optional), TAA
- [ ] **Post**: bloom, lens dirt, tonemapping, color grading
- [ ] **Weather**: rain droplets, wet track reflection, spray
- [ ] **Time of day**: dynamic sun, sky model (Preetham/Hosek-Wilkie)
- [ ] **VR**: OpenXR, foveated rendering, async reprojection
- [ ] **Triple screen**: projection correction, bezel compensation
- [ ] **GPU-driven**: culling, LOD, mesh shaders
- [ ] **Bindless** resources (VK_EXT_descriptor_indexing)

---

## 👥 Phase 7: Community & Governance (Ongoing)

### Infrastructure
- [ ] **GitHub Sponsors / Open Collective** for funding
- [ ] **Discord/Matrix** for community
- [ ] **Documentation site**: physics model, formats, API (MkDocs/GitBook)
- [ ] **Mod portal**: in-game browser, one-click install
- [ ] **Server listing**: master server (open source, self-hostable)

### Governance
- [ ] **Core team** → **Steering committee** → **Foundation** (Software Freedom Conservancy?)
- [ ] **RFC process** for major changes
- [ ] **Release cadence**: monthly patches, quarterly minors
- [ ] **LTS branches** for league stability

### Events & Growth
- [ ] **Community races** weekly
- [ ] **Content bounties** (NLnet, Epic MegaGrants)
- [ ] **League partnerships**: official server hosting
- [ ] **Hardware vendor** partnerships (wheel bases, motion)

---

## 🎯 Immediate Next Actions (This Week)

| Priority | Task | Owner | Notes |
|----------|------|-------|-------|
| 1 | Compile shaders, integrate renderer in `main.cpp` | - | Unblocks everything |
| 2 | Add physics regression test to CI | - | Prevents regressions |
| 3 | Define `car.toml` + `track.json` schemas | - | Needed for content |
| 4 | Write Blender track exporter skeleton | - | Python script |
| 5 | Publish v0.1 "ugly but drivable" demo | - | Attracts contributors |
| 6 | Open GitHub Discussions for tire/FFB/modding | - | Community input early |

---

## 📝 Progress Tracking

### Legend
- ✅ Done
- 🔄 In Progress
- ⏳ Pending
- ❌ Blocked
- 💡 Idea / Needs Discussion

### Current Sprint Focus
> **Sprint 1 (Week 1-2)**: Shader compilation → Renderer integration → First frame → Physics regression CI → car.toml schema

---

*Last updated: 2025-XX-XX*
*Next review: After Phase 1 demo*

---

## 📚 Reference Documents

- [Race Car Vehicle Dynamics (Milliken)](https://www.sae.org/publications/books/content/r-146/)
- [Tire and Vehicle Dynamics (Pacejka)](https://www.elsevier.com/books/tire-and-vehicle-dynamics/pacejka/978-0-08-097016-5)
- [Gaffer on Games - Fix Your Timestep](https://gafferongames.com/post/fix_your_timestep/)
- [Gaffer on Games - Networked Physics](https://gafferongames.com/post/networked_physics/)
- [OpenCRG Track Format](https://www.opencrg.org/)
- [GameNetworkingSockets](https://github.com/ValveSoftware/GameNetworkingSockets)
- [miniaudio](https://github.com/mackron/miniaudio)
- [Jolt Physics](https://github.com/jrouwe/JoltPhysics)

---

## 🔗 Related Issues

- [ ] #1: Shader compilation pipeline
- [ ] #2: Renderer integration in main.cpp
- [ ] #3: car.toml schema definition
- [ ] #4: Blender exporter
- [ ] #5: CI physics regression
- [ ] #6: FFB thread implementation
- [ ] #7: Multiplayer architecture