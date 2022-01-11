#version 320 es
precision highp float;

layout(location = 0) in vec3 v_normal;

layout(location = 0) out vec4 o_color;

layout(set = 0, binding = 0) uniform GlobalUniform{
    mat4 model;
    mat4 viewProj;
    vec4 rSH0;
    vec4 rSH1;
    vec4 gSH0;
    vec4 gSH1;
    vec4 bSH0;
    vec4 bSH1;
    vec4 rgbSH2;
};

vec3 ConstructSH9(vec3 normal){
    const vec4 shBias = vec4(0.282094791773878143474, 0.48860251190291992, 1.09254843059207907, 0.31539156525252); //sqrt(1/4pi), sqrt(3/4pi), sqrt(15/4pi), sqrt(5/16pi)

    vec3 sqv = normal * normal;
    vec4 t = vec4(normal.yzx, 1.0) * vec4(-shBias.y, shBias.y, -shBias.y, shBias.x);
    vec3 c = vec3(dot(t, rSH0), dot(t, gSH0), dot(t, bSH0));
    t = normal.yzzx * normal.xyzz;
    t.z = 3.0 * t.z - 1.0;
    t *= vec4(shBias.z, -shBias.z, shBias.w, -shBias.z);
    c += vec3(dot(t, rSH1), dot(t, gSH1), dot(t, bSH1));
    c += rgbSH2.xyz * (sqv.x - sqv.y) * shBias.z * 0.5;
    return c;
}

void main(){
    o_color = vec4(ConstructSH9(normalize(v_normal)), 1.0);
    // o_color = vec4(normalize(v_normal), 1.0);
}