#version 320 es
precision highp float;

layout(location = 0) in vec2 v_uv;

layout(location = 0) out vec4 o_color;

layout(set = 0, binding = 0) uniform Uniforms{
    vec4 extent;
    float targetLuma;
    float adaptationRate;
    float minExposure;
    float maxExposure;
    uint pixelCount;
};

layout(set = 0, std430, binding = 1) readonly buffer ExposureBuffer{
    // from 0 to the end
    // exposure, 1.0/exposure, exposure, 0.0, initMinLog, initMaxLog, range, rcpRange
    float exposureBuffer[];
};

layout(set = 0, binding = 2) uniform sampler2D sceneColor;

float RGBToLuminance(vec3 rgb){
    return dot(rgb, vec3(0.212671, 0.715160, 0.072169));
}

void main(){
    vec2 uv = v_uv;
    vec2 offset = extent.zw * 1.0;
    // sampler must be clamp to edge
    // sample 4x4 grid, calc luma average
    vec3 sampleColor0 = texture(sceneColor, uv + vec2(-offset.x, -offset.y)).rgb;
    vec3 sampleColor1 = texture(sceneColor, uv + vec2(-offset.x, offset.y)).rgb;
    vec3 sampleColor2 = texture(sceneColor, uv + vec2(offset.x, -offset.y)).rgb;
    vec3 sampleColor3 = texture(sceneColor, uv + offset).rgb;
    float luma = RGBToLuminance(sampleColor0 + sampleColor1 + sampleColor2 + sampleColor3) * 0.25;

    if(luma == 0.0){
        o_color = vec4(0.0);
    }else{
        float minLog = exposureBuffer[4];
        float rcpLogRange = exposureBuffer[7];
        float logLuma = clamp((log2(luma) - minLog) * rcpLogRange, 0.0, 1.0);
        o_color = vec4((logLuma * 254.0 + 1.0) / 255.0);
    }
}