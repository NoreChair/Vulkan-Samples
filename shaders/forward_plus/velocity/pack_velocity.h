
uint f32tof16(float val)
{
    uint f32 = floatBitsToUint(val);
    uint f16 = 0u;
    uint sign = (f32 >> 16) & 0x8000u;
    int exponent = int((f32 >> 23) & 0xFFu) - 127;
    uint mantissa = f32 & 0x007FFFFFu;
    if (exponent == 128)
    {
        // Infinity or NaN
        // NaN bits that are masked out by 0x3FF get discarded.
        // This can turn some NaNs to infinity, but this is allowed by the spec.
        f16 = sign | (0x1Fu << 10);
        f16 |= (mantissa & 0x3FFu);
    }
    else if (exponent > 15)
    {
        // Overflow - flush to Infinity
        f16 = sign | (0x1Fu << 10);
    }
    else if (exponent > -15)
    {
        // Representable value
        exponent += 15;
        mantissa >>= 13;
        f16 = sign | uint(exponent << 10) | mantissa;
    }
    else
    {
        f16 = sign;
    }
    return f16;
}

float f16tof32(uint val)
{
    uint sign = (val & 0x8000u) << 16;
    int exponent = int((val & 0x7C00u) >> 10);
    uint mantissa = val & 0x03FFu;
    float f32 = 0.0;
    if(exponent == 0)
    {
        if (mantissa != 0u)
        {
            const float scale = 1.0 / (1 << 24);
            f32 = scale * mantissa;
        }
    }
    else if (exponent == 31)
    {
        return uintBitsToFloat(sign | 0x7F800000u | mantissa);
    }
    else
    {
        exponent -= 15;
        float scale;
        if(exponent < 0)
        {
            // The negative unary operator is buggy on OSX.
            // Work around this by using abs instead.
            scale = 1.0 / (1 << abs(exponent));
        }
        else
        {
            scale = 1 << exponent;
        }
        float decimal = 1.0 + float(mantissa) / float(1 << 10);
        f32 = scale * decimal;
    }

    if (sign != 0u)
    {
        f32 = -f32;
    }

    return f32;
}

uint PackXY(float x){
    uint signbit = floatBitsToUint(x) >> 31;
    x = clamp(abs(x / 32768.0), 0.0, uintBitsToFloat(0x3BFFE000));
    return (f32tof16(x) + 8) >> 4 | signbit << 9;
}

uint PackZ( float x )
{
    uint signbit = floatBitsToUint(x) >> 31;
    x = clamp(abs(x / 128.0), 0, uintBitsToFloat(0x3BFFE000));
    return (f32tof16(x) + 2) >> 2 | signbit << 11;
}

// Pack the velocity to write to R10G10B10A2_UNORM
uint PackVelocity( vec3 velocity )
{
    return PackXY(velocity.x) | PackXY(velocity.y) << 10 | PackZ(velocity.z) << 20;
}

float UnpackXY(uint x)
{
    return f16tof32((x & 0x1FF) << 4 | (x >> 9) << 15) * 32768.0;
}

float UnpackZ( uint x )
{
    return f16tof32((x & 0x7FF) << 2 | (x >> 11) << 15) * 128.0;
}

vec3 UnPackVelocity(uint pack){
    return vec3(UnpackXY(pack & 0x3FF), UnpackXY((pack >> 10) & 0x3FF), UnpackZ(pack >> 20));
}