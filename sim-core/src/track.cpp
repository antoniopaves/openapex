// SPDX-FileCopyrightText: 2026 OpenApex contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "openapex/sim/track.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <vector>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace openapex::sim {

// Helper: conversión Surface desde string
Surface surface_from_string(const std::string& s) {
    if (s == "asphalt" || s == "Asphalt") return Surface::Asphalt;
    if (s == "kerb" || s == "Kerb") return Surface::Kerb;
    if (s == "grass" || s == "Grass") return Surface::Grass;
    return Surface::Asphalt;
}

std::string surface_to_string(Surface s) {
    switch (s) {
    case Surface::Asphalt: return "asphalt";
    case Surface::Kerb:    return "kerb";
    case Surface::Grass:   return "grass";
    }
    return "asphalt";
}

// Constructor principal desde TrackPoints
Track::Track(std::vector<TrackPoint> points) : pts_(std::move(points)) {
    if (pts_.size() < 2) {
        throw std::runtime_error("Track must have at least 2 points");
    }
    rebuild_cumulative();
}

// Constructor legacy para compatibilidad
Track::Track(std::vector<Vec2> centerline, double width, double kerb_width)
    : pts_() {
    pts_.reserve(centerline.size());
    for (const auto& p : centerline) {
        TrackPoint tp;
        tp.pos = p;
        tp.width = width;
        tp.kerb_width = kerb_width;
        pts_.push_back(tp);
    }
    rebuild_cumulative();
}

void Track::rebuild_cumulative() noexcept {
    const std::size_t n = pts_.size();
    centerline_.clear();
    centerline_.reserve(n);
    for (const auto& p : pts_) {
        centerline_.push_back(p.pos);
    }
    cumulative_.resize(n);
    segment_lengths_.resize(n);
    cumulative_[0] = 0.0;
    length_ = 0.0;

    for (std::size_t i = 1; i < n; ++i) {
        const double dx = pts_[i].pos.x - pts_[i - 1].pos.x;
        const double dy = pts_[i].pos.y - pts_[i - 1].pos.y;
        const double seg_len = std::hypot(dx, dy);
        segment_lengths_[i - 1] = seg_len;
        cumulative_[i] = cumulative_[i - 1] + seg_len;
        length_ += seg_len;
    }
    // Último segmento: cierre del bucle (último punto al primero)
    const double dx = pts_[0].pos.x - pts_[n - 1].pos.x;
    const double dy = pts_[0].pos.y - pts_[n - 1].pos.y;
    const double seg_len = std::hypot(dx, dy);
    segment_lengths_[n - 1] = seg_len;
    length_ += seg_len;
}

Track Track::make_demo() {
    std::vector<TrackPoint> points;
    points.reserve(400);

    const double straight_len = 400.0;
    const double curve_radius = 150.0;
    const int straight_segments = 40;
    const int curve_segments = 60;
    const double start_x = -straight_len / 2.0;  // -200
    const double start_y = 0.0;

    // First straight: (-200, 0) to (200, 0), heading east
    for (int i = 0; i <= straight_segments; ++i) {
        TrackPoint tp;
        tp.pos = {start_x + i * straight_len / straight_segments, start_y};
        tp.width = 12.0;
        tp.kerb_width = 1.5;
        tp.surface_left = Surface::Asphalt;
        tp.surface_right = Surface::Asphalt;
        points.push_back(tp);
    }

    // First curve (left turn, northward): center at (200, 150), angle PI to 2*PI
    const double cx1 = start_x + straight_len;  // 200
    const double cy1 = curve_radius;            // 150
    for (int i = 1; i <= curve_segments; ++i) {
        const double angle = M_PI + i * M_PI / curve_segments;
        TrackPoint tp;
        tp.pos = {
            cx1 - curve_radius * std::sin(angle),
            cy1 + curve_radius * std::cos(angle)
        };
        tp.width = 12.0;
        tp.kerb_width = 1.5;
        // Banking en curva: inclinación hacia el interior (izquierda en curva izquierda)
        tp.banking = 0.15; // ~8.6 grados
        points.push_back(tp);
    }

    // Second straight: (200, 300) to (-200, 300), heading west
    for (int i = 1; i <= straight_segments; ++i) {
        TrackPoint tp;
        tp.pos = {cx1 - i * straight_len / straight_segments, cy1 + curve_radius};
        tp.width = 12.0;
        tp.kerb_width = 1.5;
        points.push_back(tp);
    }

    // Second curve (left turn, southward): center at (-200, 150), angle 0 to PI
    const double cx2 = start_x;                 // -200
    const double cy2 = curve_radius;            // 150
    for (int i = 1; i <= curve_segments; ++i) {
        const double angle = i * M_PI / curve_segments;
        TrackPoint tp;
        tp.pos = {
            cx2 - curve_radius * std::sin(angle),
            cy2 + curve_radius * std::cos(angle)
        };
        tp.width = 12.0;
        tp.kerb_width = 1.5;
        tp.banking = 0.15;
        points.push_back(tp);
    }

    Track track(std::move(points));
    track.meta_ = {"Demo Circuit", "OpenApex", "Default demo track", 1};
    track.pit_lane_ = {track.length() * 0.1, track.length() * 0.15, 80.0 / 3.6};
    return track;
}

Track Track::load_from_file(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open track file: " + path);
    }

    json j;
    try {
        file >> j;
    } catch (const json::parse_error& e) {
        throw std::runtime_error("JSON parse error in " + path + ": " + e.what());
    }

    // Validar versión
    int version = j.value("version", 1);
    if (version != 1) {
        throw std::runtime_error("Unsupported track file version: " + std::to_string(version));
    }

    // Leer metadatos
    TrackMetadata meta;
    if (j.contains("metadata")) {
        const auto& m = j["metadata"];
        meta.name = m.value("name", "Unnamed Track");
        meta.author = m.value("author", "Unknown");
        meta.description = m.value("description", "");
        meta.version = m.value("version", 1);
    }

    // Leer geometría
    if (!j.contains("geometry") || !j["geometry"].contains("centerline")) {
        throw std::runtime_error("Track file missing geometry.centerline");
    }

    const auto& centerline = j["geometry"]["centerline"];
    if (!centerline.is_array() || centerline.size() < 2) {
        throw std::runtime_error("Track centerline must have at least 2 points");
    }

    std::vector<TrackPoint> points;
    points.reserve(centerline.size());

    for (std::size_t i = 0; i < centerline.size(); ++i) {
        const auto& p = centerline[i];
        TrackPoint tp;
        tp.pos.x = p.value("x", 0.0);
        tp.pos.y = p.value("y", 0.0);
        tp.width = p.value("width", 12.0);
        tp.kerb_width = p.value("kerb_width", 1.5);
        tp.elevation = p.value("elevation", 0.0);
        tp.banking = p.value("banking", 0.0);
        tp.surface_left = surface_from_string(p.value("surface_left", "asphalt"));
        tp.surface_right = surface_from_string(p.value("surface_right", "asphalt"));
        points.push_back(tp);
    }

    Track track(std::move(points));
    track.meta_ = meta;

    // Leer objetos
    if (j.contains("objects") && j["objects"].is_array()) {
        for (const auto& obj : j["objects"]) {
            TrackObject to;
            std::string type_str = obj.value("type", "cone");
            if (type_str == "cone") to.type = TrackObject::Type::Cone;
            else if (type_str == "tire_barrier") to.type = TrackObject::Type::TireBarrier;
            else if (type_str == "armco_barrier") to.type = TrackObject::Type::ArmcoBarrier;
            else if (type_str == "grandstand") to.type = TrackObject::Type::Grandstand;
            else if (type_str == "building") to.type = TrackObject::Type::Building;
            else if (type_str == "tree") to.type = TrackObject::Type::Tree;

            const json& pos = obj.contains("position") && obj["position"].is_object() ? obj["position"] : obj;
            to.position.x = pos.value("x", 0.0);
            to.position.y = pos.value("y", 0.0);
            to.rotation = obj.value("rotation", 0.0);
            to.scale = obj.value("scale", 1.0);
            track.objects_.push_back(to);
        }
    }

    // Leer pit lane
    if (j.contains("pit_lane")) {
        const auto& pl = j["pit_lane"];
        track.pit_lane_.entry_along = pl.value("entry", 0.0);
        track.pit_lane_.exit_along = pl.value("exit", 0.0);
        track.pit_lane_.speed_limit = pl.value("speed_limit", 80.0 / 3.6);
    }

    // Validar pista
    if (auto err = track.validate()) {
        throw std::runtime_error("Track validation failed: " + *err);
    }

    return track;
}

void Track::save_to_file(const std::string& path) const {
    json j;
    j["version"] = meta_.version;
    j["metadata"] = {
        {"name", meta_.name},
        {"author", meta_.author},
        {"description", meta_.description},
        {"version", meta_.version}
    };

    json centerline = json::array();
    for (const auto& p : pts_) {
        centerline.push_back({
            {"x", p.pos.x},
            {"y", p.pos.y},
            {"width", p.width},
            {"kerb_width", p.kerb_width},
            {"elevation", p.elevation},
            {"banking", p.banking},
            {"surface_left", surface_to_string(p.surface_left)},
            {"surface_right", surface_to_string(p.surface_right)}
        });
    }
    j["geometry"]["centerline"] = centerline;

    json objects = json::array();
    for (const auto& obj : objects_) {
        std::string type_str;
        switch (obj.type) {
            case TrackObject::Type::Cone: type_str = "cone"; break;
            case TrackObject::Type::TireBarrier: type_str = "tire_barrier"; break;
            case TrackObject::Type::ArmcoBarrier: type_str = "armco_barrier"; break;
            case TrackObject::Type::Grandstand: type_str = "grandstand"; break;
            case TrackObject::Type::Building: type_str = "building"; break;
            case TrackObject::Type::Tree: type_str = "tree"; break;
        }
        objects.push_back({
            {"type", type_str},
            {"position", {{"x", obj.position.x}, {"y", obj.position.y}}},
            {"rotation", obj.rotation},
            {"scale", obj.scale}
        });
    }
    j["objects"] = objects;

    j["pit_lane"] = {
        {"entry", pit_lane_.entry_along},
        {"exit", pit_lane_.exit_along},
        {"speed_limit", pit_lane_.speed_limit}
    };

    std::ofstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot write track file: " + path);
    }
    file << j.dump(2); // Pretty print with 2-space indent
}

TrackQuery Track::query(double x, double y) const noexcept {
    TrackQuery result;
    const std::size_t n = pts_.size();

    if (n < 2) {
        return result;
    }

    double min_dist_sq = std::numeric_limits<double>::infinity();
    double closest_along = 0.0;
    double closest_lateral = 0.0;
    double closest_heading = 0.0;
    std::size_t closest_segment = 0;

    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t next = (i + 1) % n;
        const Vec2& a = pts_[i].pos;
        const Vec2& b = pts_[next].pos;

        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double seg_len_sq = dx * dx + dy * dy;

        double t = 0.0;
        if (seg_len_sq > 0.0) {
            t = std::clamp(((x - a.x) * dx + (y - a.y) * dy) / seg_len_sq, 0.0, 1.0);
        }

        const double proj_x = a.x + t * dx;
        const double proj_y = a.y + t * dy;
        const double dist_sq = (x - proj_x) * (x - proj_x) + (y - proj_y) * (y - proj_y);

        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;

            const double seg_len = std::sqrt(seg_len_sq);
            const double along_seg = t * seg_len;
            const double along = cumulative_[i] + along_seg;

            const double seg_dx = dx / seg_len;
            const double seg_dy = dy / seg_len;
            const double lat = (x - proj_x) * (-seg_dy) + (y - proj_y) * seg_dx;

            const double heading = std::atan2(dy, dx);

            closest_along = along;
            closest_lateral = lat;
            closest_heading = heading;
            closest_segment = i;
        }
    }

    result.along = closest_along;
    result.lateral = closest_lateral;
    result.heading = closest_heading;

    // Interpolar elevación y banking del segmento más cercano
    const std::size_t next = (closest_segment + 1) % n;
    const double seg_len = segment_lengths_[closest_segment];
    if (seg_len > 0.0) {
        const double t = (closest_along - cumulative_[closest_segment]) / seg_len;
        result.elevation = pts_[closest_segment].elevation + t * (pts_[next].elevation - pts_[closest_segment].elevation);
        result.banking = pts_[closest_segment].banking + t * (pts_[next].banking - pts_[closest_segment].banking);
    } else {
        result.elevation = pts_[closest_segment].elevation;
        result.banking = pts_[closest_segment].banking;
    }

    // Determinar superficie basada en lateral y segmento
    result.surface = surface_at(closest_lateral, closest_segment);

    return result;
}

Surface Track::surface_at(double lateral, std::size_t segment_idx) const noexcept {
    const std::size_t n = pts_.size();
    if (segment_idx >= n) segment_idx = n - 1;

    const double half_width = pts_[segment_idx].width * 0.5;
    const double kerb_outer = half_width + pts_[segment_idx].kerb_width;
    const double abs_lat = std::abs(lateral);

    // Usar superficie izquierda o derecha según el lado
    const Surface left_surf = pts_[segment_idx].surface_left;
    const Surface right_surf = pts_[segment_idx].surface_right;

    // Interpolar superficies? Por simplicidad, usar la del segmento actual
    if (lateral >= 0) {
        // Lado izquierdo
        if (abs_lat <= half_width) return left_surf;
        if (abs_lat <= kerb_outer) return Surface::Kerb;
        return Surface::Grass;
    } else {
        // Lado derecho
        if (abs_lat <= half_width) return right_surf;
        if (abs_lat <= kerb_outer) return Surface::Kerb;
        return Surface::Grass;
    }
}

std::optional<std::string> Track::validate() const noexcept {
    const std::size_t n = pts_.size();
    if (n < 3) {
        return "Track must have at least 3 points";
    }

    // El bucle se cierra solo con el segmento ultimo->primero. Solo comprobamos que ese
    // segmento de cierre no sea absurdamente mas largo que el resto de la pista.
    const double closing = segment_lengths_[n - 1];
    double longest = 0.0;
    for (std::size_t i = 0; i + 1 < n; ++i) {
        longest = std::max(longest, segment_lengths_[i]);
    }
    if (closing > 1.5 * longest + 1e-9) {
        return "Track does not close properly (closing segment " + std::to_string(closing) +
               "m, longest other segment " + std::to_string(longest) + "m)";
    }

    // Verificar anchos positivos
    for (std::size_t i = 0; i < n; ++i) {
        if (pts_[i].width <= 0.0) {
            return "Invalid width at point " + std::to_string(i);
        }
        if (pts_[i].kerb_width < 0.0) {
            return "Invalid kerb width at point " + std::to_string(i);
        }
    }

    return std::nullopt; // Válida
}

double Track::width_at(std::size_t index) const noexcept {
    if (index >= pts_.size()) return pts_.back().width;
    return pts_[index].width;
}

double Track::kerb_width_at(std::size_t index) const noexcept {
    if (index >= pts_.size()) return pts_.back().kerb_width;
    return pts_[index].kerb_width;
}

const std::vector<Vec2>& Track::centerline() const noexcept { return centerline_; }

Vec2 Track::start_position() const {
    return pts_.empty() ? Vec2{} : pts_[0].pos;
}

double Track::start_heading() const {
    if (pts_.size() < 2) return 0.0;
    const Vec2& a = pts_[0].pos;
    const Vec2& b = pts_[1].pos;
    return std::atan2(b.y - a.y, b.x - a.x);
}

double surface_grip(Surface surface) noexcept {
    switch (surface) {
    case Surface::Asphalt: return 1.0;
    case Surface::Kerb:    return 0.7;
    case Surface::Grass:   return 0.3;
    }
    return 0.3;
}

} // namespace openapex::sim