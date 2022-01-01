#version 320 es
precision highp float;

layout(location = 0) in vec3 v_normal;

layout(location = 0) out vec4 o_color;

layout(set = 0, binding = 0) uniform GlobalUniform{
    mat4 viewProj;
    vec4 sunDir;
    // vec4 cameraPos;
    // vec4 projOrigin;
    // vec4 projPlaneU;
    // vec4 projPlaneV;
    // vec4 viewport;
};

void main(){
    vec3 normal = normalize(v_normal);
    // vec2 uv = gl_FragCoord.xy * viewport.zw;
    // vec3 projPoint = projOrigin.xyz + uv.x * projPlaneU.xyz + uv.y * projPlaneV.xyz;
    // vec3 viewDir = normalize(projPoint - cameraPos);
    vec3 sky = vec3(0.5, 0.5, 0.9) - max(normal.y, 0.0) * 0.3;
    vec3 sun = pow(max(dot(normal, -sunDir.xyz), 0.0), 100.0) * vec3(1.0, 0.9, 0.7);
    o_color = vec4(sun + sky, 1.0);
}