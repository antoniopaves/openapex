# OpenApex Implementation Plan

## Overview
Three major features to implement:
1. **Track file format & loader** - Enable content creation
2. **ABS, Traction Control, LSD** - Advanced vehicle dynamics
3. **Vulkan Renderer** - Modern GPU rendering pipeline

---

## Phase 1: Track File Format & Loader (Priority: High)

### 1.1 Track File Format Design
**Format**: JSON (human-readable, versionable, easy to edit)
**Extension**: `.track.json`

```json
{
  "version": 1,
  "metadata": {
    "name": "Circuit Example",
    "author": "Track Designer",
    "description": "A demo circuit"
  },
  "geometry": {
    "centerline": [
      {"x": -200.0, "y": 0.0, "width": 12.0, "kerb_width": 1.5, "banking": 0.0},
      {"x": -190.0, "y": 0.0, "width": 12.0, "kerb_width": 1.5, "banking": 0.0}
    ]
  },
  "surfaces": [
    {"start": 0, "end": 100, "left": "asphalt", "right": "asphalt"},
    {"start": 100, "end": 200, "left": "kerb", "right": "grass"}
  ],
  "objects": [
    {"type": "cone", "position": {"x": 10, "y": 5}, "rotation": 0.0}
  ],
  "start_position": {"x": -200.0, "y": 0.0, "heading": 0.0},
  "pit_lane": {"entry": 500, "exit": 550, "speed_limit": 80}
}
```

### 1.2 Implementation Tasks

| Task | Description | Files |
|------|-------------|-------|
| 1.2.1 | Add track file structs to `track.hpp` | `sim-core/include/openapex/sim/track.hpp` |
| 1.2.2 | Implement `Track::load_from_file()` | `sim-core/src/track.cpp` |
| 1.2.3 | Add JSON parsing (use nlohmann/json or rapidjson) | `sim-core/CMakeLists.txt`, `vcpkg.json` |
| 1.2.4 | Add surface type per-segment support | `track.hpp`, `track.cpp` |
| 1.2.5 | Add elevation & banking support | `track.hpp`, `track.cpp` |
| 1.2.6 | Create track validation (closed loop, continuity) | `track.cpp` |
| 1.2.7 | Add `Track::save_to_file()` for editor | `track.cpp` |
| 1.2.8 | Create example track files in `data/tracks/` | `data/tracks/*.track.json` |
| 1.2.9 | Update demo to load track from file | `app/main.cpp` |
| 1.2.10 | Add unit tests for loader | `sim-core/tests/test_track.cpp` |

### 1.3 Dependencies
- Add `nlohmann/json` to `vcpkg.json`
- Or use `rapidjson` (header-only, faster)

---

## Phase 2: ABS, Traction Control & LSD (Priority: High)

### 2.1 VehicleParams Extensions
Add to `VehicleParams` struct in `vehicle.hpp`:

```cpp
// ABS
bool abs_enabled = true;
double abs_slip_threshold = 0.15;      // Optimal slip ratio for peak braking
double abs_modulation_rate = 10.0;     // How fast to modulate (Hz)

// Traction Control
bool tc_enabled = true;
double tc_slip_threshold = 0.08;       // Optimal slip ratio for acceleration
double tc_torque_reduction_rate = 5.0; // Torque reduction speed

// LSD (Limited Slip Differential)
enum class DiffType { Open, Viscous, Clutch, Torsen };
DiffType diff_type = DiffType::Clutch;
double diff_preload = 50.0;           // Nm base locking torque
double diff_lock_coast = 0.3;         // Lock ratio on coast (0-1)
double diff_lock_power = 0.7;         // Lock ratio on power (0-1)
double diff_ramp_angle_power = 45.0;  // Degrees
double diff_ramp_angle_coast = 30.0;  // Degrees
int diff_clutch_packs = 4;            // Number of clutch plates
```

### 2.2 VehicleState Extensions
Add to `VehicleState`:
```cpp
std::array<bool, 4> abs_active{};
bool tc_active = false;
double diff_lock_torque = 0.0;
```

### 2.3 Implementation Tasks

| Task | Description | Files |
|------|-------------|-------|
| 2.3.1 | Extend VehicleParams with ABS/TC/LSD params | `vehicle.hpp` |
| 2.3.2 | Extend VehicleState with system states | `vehicle.hpp` |
| 2.3.3 | Implement ABS logic in `step_vehicle()` | `vehicle.cpp` |
| 2.3.4 | Implement TC logic (torque reduction) | `vehicle.cpp` |
| 2.3.5 | Implement LSD torque distribution | `vehicle.cpp` |
| 2.3.6 | Add LSD types: Viscous, Clutch, Torsen | `vehicle.cpp` |
| 2.3.7 | Add input controls for toggling systems | `app/main.cpp` |
| 2.3.8 | Add telemetry display for ABS/TC/LSD | `app/main.cpp` |
| 2.3.9 | Add unit tests for each system | `sim-core/tests/test_vehicle.cpp` |

### 2.4 Algorithm Details

**ABS**: Monitor rear wheel slip ratios during braking. If slip > threshold, reduce brake pressure on that wheel proportionally.

**TC**: Monitor rear wheel slip ratios during acceleration. If slip > threshold, reduce engine torque or apply individual rear brake.

**LSD (Clutch-type)**:
- Calculate torque bias ratio based on input torque and ramp angles
- Distribute torque: `T_left = (T_input + T_lock) / 2`, `T_right = (T_input - T_lock) / 2`
- `T_lock = preload + (power/coast) * T_input * tan(ramp_angle)`

---

## Phase 3: Vulkan Renderer (Priority: Medium-High)

### 3.1 Architecture
Replace SDL3 immediate-mode rendering with Vulkan retained-mode pipeline.

### 3.2 Implementation Tasks

| Task | Description | Files |
|------|-------------|-------|
| 3.2.1 | Create `renderer/vulkan/` module structure | `renderer/vulkan/CMakeLists.txt` |
| 3.2.2 | Add Vulkan to `vcpkg.json` | `vcpkg.json` |
| 3.2.3 | Implement Vulkan context (instance, device, swapchain) | `renderer/vulkan/context.hpp/cpp` |
| 3.2.4 | Create render pass & framebuffer management | `renderer/vulkan/renderpass.hpp/cpp` |
| 3.2.5 | Implement descriptor set layout & pipeline layout | `renderer/vulkan/pipeline.hpp/cpp` |
| 3.2.6 | Create shaders: track.vert/frag, car.vert/frag, ui.vert/frag | `renderer/vulkan/shaders/` |
| 3.2.7 | Implement track mesh generation (triangulated strips) | `renderer/vulkan/track_renderer.hpp/cpp` |
| 3.2.8 | Implement car model loading (glTF) | `renderer/vulkan/model.hpp/cpp` |
| 3.2.9 | Implement camera system (view/projection matrices) | `renderer/vulkan/camera.hpp/cpp` |
| 3.2.10 | Implement UI rendering (ImGui + Vulkan backend) | `renderer/vulkan/ui.hpp/cpp` |
| 3.2.11 | Create main Vulkan renderer class | `renderer/vulkan/renderer.hpp/cpp` |
| 3.2.12 | Integrate with app (replace SDL rendering) | `app/main.cpp` |
| 3.2.13 | Add validation layers for debug builds | `renderer/vulkan/context.cpp` |
| 3.2.14 | Implement frame timing & presentation modes | `renderer/vulkan/swapchain.cpp` |

### 3.3 Shader Plan

**Track Shader** (`track.vert`/`track.frag`):
- Input: position, color, surface_type
- Vertex: transform by MVP, pass surface_type
- Fragment: color by surface (asphalt/kerb/grass), optional texture

**Car Shader** (`car.vert`/`car.frag`):
- Input: position, normal, UV
- Vertex: skinning (if animated), transform
- Fragment: PBR (albedo, metallic, roughness) or simple lit

**UI Shader** (`ui.vert`/`ui.frag`):
- Input: position, UV, color
- Standard ImGui vertex format

### 3.4 Data Flow
```
Simulation (60Hz) → State Buffer → Render Thread (Vulkan) → Swapchain → Display
```

### 3.5 Dependencies
- Vulkan SDK (via vcpkg: `vulkan-headers`, `vulkan-loader`, `vulkan-validation-layers`)
- glTF loader: `tinygltf` or `fastgltf`
- ImGui: `imgui` + `imgui-vulkan` backend
- glm (already present) for math

---

## Implementation Order Recommendation

### Sprint 1 (Week 1-2): Track System
1. Add nlohmann/json dependency
2. Design & implement track file format
3. Create 2-3 example tracks
4. Update app to load track from file

### Sprint 2 (Week 2-3): Vehicle Dynamics
1. Add ABS/TC/LSD params to VehicleParams
2. Implement each system in step_vehicle
3. Add UI toggles and telemetry
4. Test with existing demo track

### Sprint 3 (Week 3-5): Vulkan Renderer
1. Set up Vulkan context & swapchain
2. Basic triangle → track lines → track mesh
3. Car model (simple box → glTF model)
4. Camera & UI integration
5. Polish: lighting, shadows, post-process

---

## File Structure After Implementation

```
openapex/
├── app/
│   ├── main.cpp              # Updated for Vulkan + track loading + ABS/TC/LSD UI
│   └── CMakeLists.txt
├── sim-core/
│   ├── include/openapex/sim/
│   │   ├── track.hpp         # Extended with file I/O, surfaces, elevation
│   │   ├── vehicle.hpp       # Extended with ABS/TC/LSD params
│   │   └── tire.hpp
│   ├── src/
│   │   ├── track.cpp         # File loader, surface queries
│   │   ├── vehicle.cpp       # ABS/TC/LSD implementation
│   │   └── tire.cpp
│   ├── tests/
│   │   ├── test_track.cpp    # Track loading tests
│   │   └── test_vehicle.cpp  # ABS/TC/LSD tests
│   └── CMakeLists.txt
├── renderer/
│   └── vulkan/
│       ├── CMakeLists.txt
│       ├── context.hpp/cpp
│       ├── renderer.hpp/cpp
│       ├── track_renderer.hpp/cpp
│       ├── car_renderer.hpp/cpp
│       ├── ui.hpp/cpp
│       ├── camera.hpp/cpp
│       ├── pipeline.hpp/cpp
│       ├── shaders/
│       │   ├── track.vert/spv
│       │   ├── track.frag/spv
│       │   ├── car.vert/spv
│       │   ├── car.frag/spv
│       │   ├── ui.vert/spv
│       │   └── ui.frag/spv
│       └── model.hpp/cpp     # glTF loading
├── data/
│   └── tracks/
│       ├── demo.track.json
│       ├── monza.track.json
│       └── spa.track.json
├── vcpkg.json                # Updated with new deps
└── CMakeLists.txt
```

---

## Testing Strategy

1. **Unit Tests**: Each system tested in isolation (Catch2)
2. **Integration Tests**: Full simulation step with all systems active
3. **Visual Tests**: Render test tracks, verify car physics visually
4. **Regression Tests**: Compare lap times before/after changes

---

## Risks & Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| Vulkan complexity | High | Start with minimal triangle, incrementally add features |
| Track format changes | Medium | Version the format, maintain backward compatibility |
| Physics stability | Medium | Small dt (1/360), clamp forces, extensive testing |
| Performance | Low | Profile early, use RenderDoc, optimize shaders |

---

## Next Steps

1. **Immediate**: Add nlohmann/json to vcpkg.json and implement track file loader
2. **Short-term**: Implement ABS/TC/LSD in vehicle simulation
3. **Long-term**: Begin Vulkan renderer prototype