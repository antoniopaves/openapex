#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec2 inUV;

layout(push_constant) uniform PushConstants {
    mat4 mvp;
} push;

layout(location = 0) out vec3 fragNormal;
layout(location = 1) out vec2 fragUV;

void main() {
    gl_Position = push.mvp * vec4(inPosition, 1.0);
    fragNormal = inNormal;
    fragUV = inUV;
}