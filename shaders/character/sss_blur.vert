#version 320 es
precision highp float;

layout(location = 0) in vec3 position; // x from -1 to 3, y from -1 to 3

layout(location = 0) out vec2 v_uv;

layout(set = 0, binding = 0) uniform SSS{
    vec2 stepSize;
    float correction;
    float maxdd;
    float depth;
};

void main(){
    // v_uv = vec2((gl_VertexIndex << 1) & 2, gl_VertexIndex & 2);
    // gl_Position = vec4(v_uv * 2.0 - 1.0, 0.0, 1.0);

    v_uv = position.xy * 0.5 + vec2(0.5);
    gl_Position = vec4(position.xy, depth, 1.0);
}