#version 320 es
precision highp float;

layout(location = 0) in vec2 v_uv;

layout(location = 0) out vec4 o_color;

layout(set = 0, binding = 0) uniform Uniforms{
    float exposure;
};

layout(set = 0, binding = 1) uniform sampler2D hdrImage;

// The Reinhard tone operator.  Typically, the value of k is 1.0, but you can adjust exposure by 1/k.
// I.e. TM_Reinhard(x, 0.5) == TM_Reinhard(x * 2.0, 1.0)

vec3 TM_Reinhard(vec3 hdr, float k /*= 1.0*/)
{
    return hdr / (hdr + k);
}

// The inverse of Reinhard
vec3 ITM_Reinhard(vec3 sdr, float k /*= 1.0*/)
{
    return k * sdr / (k - sdr);
}

vec3 ToneMapACES(vec3 hdr)
{
    const float A = 2.51, B = 0.03, C = 2.43, D = 0.59, E = 0.14;
    return clamp((hdr * (A * hdr + B)) / (hdr * (C * hdr + D) + E), vec3(0.0), vec3(1.0));
}

vec3 InverseToneMapACES(vec3 sdr)
{
    const float A = 2.51, B = 0.03, C = 2.43, D = 0.59, E = 0.14;
    return 0.5 * (D * sdr - sqrt(((D*D - 4.0*C*E) * sdr + 4.0*A*E-2.0*B*D) * sdr + B*B) - B) / (A - C * sdr);
}

// This is the new tone operator.  It resembles ACES in many ways, but it is simpler to evaluate with ALU.  One
// advantage it has over Reinhard-Squared is that the shoulder goes to white more quickly and gives more overall
// brightness and contrast to the image.
// https://www.desmos.com/calculator/sqjhtq4rot
vec3 ToneMapping_Standard(vec3 hdr){
    return TM_Reinhard(hdr * sqrt(hdr), sqrt(4.0 / 27.0));
}

void main(){
    vec4 hdrColor = texture(hdrImage, v_uv);
    vec3 sdrColor = ToneMapping_Standard(hdrColor.rgb * exposure);
    // vec3 sdrColor = ToneMapACES(hdrColor.rgb * exposure);
    o_color = vec4(sdrColor, 1.0);
}