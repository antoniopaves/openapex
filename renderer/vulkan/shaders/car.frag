#version 450

layout(location = 0) in vec3 fragNormal;
layout(location = 1) in vec2 fragUV;

layout(location = 0) out vec4 outColor;

void main() {
    // Car body color - red
    vec3 albedo = vec3(0.8, 0.1, 0.1);

    // Simple lighting
    vec3 light_dir = normalize(vec3(0.5, 1.0, 0.3));
    float diff = max(dot(fragNormal, light_dir), 0.3);

    vec3 color = albedo * diff;

    outColor = vec4(color, 1.0);
}