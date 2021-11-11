#version 320 es
precision highp float;

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 translate;
layout(location = 2) in vec3 scale;
layout(location = 3) in vec4 rotation;

layout(set = 0, binding = 0) uniform GlobalUniform{
    mat4 viewProj;
};

mat4 get_model_matrix(vec3 t, vec3 s, vec4 q){
    mat4 model;

    float qxx = q.x * q.x;
    float qyy = q.y * q.y;
    float qzz = q.z * q.z;
    float qxz = q.x * q.z;
    float qxy = q.x * q.y;
    float qyz = q.y * q.z;
    float qwx = q.w * q.x;
    float qwy = q.w * q.y;
    float qwz = q.w * q.z;

    model[0][0] = 1.0 - 2.0 * (qyy +  qzz);
    model[0][1] = 2.0 * qxy + qwz;
    model[0][2] = 2.0 * qxz - qwy;
    model[0][3] = 0.0;

    model[1][0] = 2.0 * qxy - qwz;
    model[1][1] = 1.0 - 2.0 * (qxx +  qzz);
    model[1][2] = 2.0 * qyz + qwx;
    model[1][3] = 0.0;

    model[2][0] = 2.0 * qxz + qwy;
    model[2][1] = 2.0 * qyz - qwx;
    model[2][2] = 1.0 - 2.0 * (qxx +  qyy);
    model[2][3] = 0.0;

    model[0][0] *= scale.x;
    model[1][1] *= scale.y;
    model[2][2] *= scale.z;
    model[3] = vec4(t, 0.0);

    return model;
}

void main(){
    mat4 model = get_model_matrix(translate, scale, rotation);
    gl_Position = viewProj * model * vec4(position, 1.0);
}
