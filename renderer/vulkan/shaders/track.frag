#version 450

layout(location = 0) in vec3 fragNormal;
layout(location = 1) in vec2 fragUV;
layout(location = 2) in uint fragSurfaceType;

layout(location = 0) out vec4 outColor;

void main() {
    vec3 color;

    // Color based on surface type
    if (fragSurfaceType == 0u) {
        // Asphalt - dark gray
        color = vec3(0.2, 0.2, 0.22);
    } else if (fragSurfaceType == 1u) {
        // Kerb - red/white striped (simplified to red)
        color = vec3(0.8, 0.1, 0.1);
    } else {
        // Grass - green
        color = vec3(0.1, 0.5, 0.1);
    }

    // Simple lighting
    vec3 light_dir = normalize(vec3(0.5, 1.0, 0.3));
    float diff = max(dot(fragNormal, light_dir), 0.2);
    color *= diff;

    outColor = vec4(color, 1.0);
}