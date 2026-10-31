// SPDX-FileCopyrightText: 2026 OpenApex contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "context.hpp"
#include "track_renderer.hpp"
#include "car_renderer.hpp"
#include "ui.hpp"
#include "camera.hpp"

#include <openapex/sim/track.hpp>
#include <openapex/sim/vehicle.hpp>

#include <memory>

namespace openapex::renderer::vulkan {

class Renderer {
public:
    Renderer() = default;
    ~Renderer();

    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;
    Renderer(Renderer&&) noexcept = default;
    Renderer& operator=(Renderer&&) noexcept = default;

    // Initialize the renderer
    bool init(SDL_Window* window);

    // Render a frame
    bool render_frame(
        const openapex::sim::Track& track,
        const openapex::sim::VehicleState& vehicle_state,
        const openapex::sim::VehicleParams& vehicle_params,
        const Camera& camera,
        const UIState& ui_state);

    // Handle window resize
    void on_resize(SDL_Window* window);

    // Wait for GPU idle
    void wait_idle() const;

    // Getters
    [[nodiscard]] const Context& context() const noexcept { return context_; }
    [[nodiscard]] Context& context() noexcept { return context_; }
    [[nodiscard]] const TrackRenderer& track_renderer() const noexcept { return track_renderer_; }
    [[nodiscard]] const CarRenderer& car_renderer() const noexcept { return car_renderer_; }
    [[nodiscard]] const UIRenderer& ui_renderer() const noexcept { return ui_renderer_; }

private:
    Context context_;
    TrackRenderer track_renderer_;
    CarRenderer car_renderer_;
    UIRenderer ui_renderer_;
    Camera camera_;
    Pipeline track_pipeline_;
    Pipeline car_pipeline_;
};

} // namespace openapex::renderer::vulkan