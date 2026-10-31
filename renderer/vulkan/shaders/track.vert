#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec2 inUV;
layout(location = 3) in uint inSurfaceType;

layout(push_constant) uniform PushConstants {
    mat4 mvp;
} push;

layout(location = 0) out vec3 fragNormal;
layout(location = 1) out vec2 fragUV;
layout(location = 2) out uint fragSurfaceType;

void main() {
    gl_Position = push.mvp * vec4(inPosition, 1.0);
    fragNormal = inNormal;
    fragUV = inUV;
    fragSurfaceType = inSurfaceType;
}