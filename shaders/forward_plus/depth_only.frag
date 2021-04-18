#version 320 es
precision highp float;

layout(location = 0) out vec4 o_color;

layout(set = 0, binding = 0) uniform GlobalUniform{
    mat4 model;
    mat4 viewProj;
    vec3 cameraPosition;
};

void main(){
    float depth = 1.0 / gl_FragCoord.w;
    depth /= 4000.0;
    o_color = vec4(depth, depth, depth, 1.0);
}