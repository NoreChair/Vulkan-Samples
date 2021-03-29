#define MAX_LIGHTS 128
#define TILE_SIZE (4 + MAX_LIGHTS * 4)
#define FLT_MIN         1.175494351e-38F        // min positive value
#define FLT_MAX         3.402823466e+38F        // max value
#define PI                3.1415926535f
#define TWOPI            6.283185307f

// https://developer.nvidia.com/content/understanding-structured-buffer-performance
// for performance reason, data must align with 128-bit
struct LightData{
    float4 position; // xyz=pos, w=radius
    float4 color; // xyz=rgb, w=intensity
    float4 coneDir; // xyz=direction, w=range
    float3 coneAngles; // x=1.0f/(cos(inner)-cos(outer)), y=cos(inner), z=cos(inner/2)
    int lightType;
    float4x4 shadowTextureMatrix;
};

cbuffer LightGridCB : register(b0) {
    uint2 viewport;
    float invTileDim;
    float rcpZMagic; // near / (far - near)
    uint tileCountX;
    float4x4 viewProjMatrix;
};

StructuredBuffer<LightData> lightBuffer;
Texture2D<float> depthTexture; // linear depth
RWByteAddressBuffer lightGrid;
RWByteAddressBuffer lightGridBitMask;

groupshared uint minDepthUInt;
groupshared uint maxDepthUInt;
groupshared uint tileLightCountSphere;
groupshared uint tileLightCountCone;
groupshared uint tileLightCountConeShadowed;
groupshared uint tileLightIndicesSphere[MAX_LIGHTS];
groupshared uint tileLightIndicesCone[MAX_LIGHTS];
groupshared uint tileLightIndicesConeShadowed[MAX_LIGHTS];
groupshared uint4 tileLightBitMask;

uint2 GetTilePos(float2 pos, float2 invTileDim){
    return (uint2)floor(pos * invTileDim);
}

uint GetTileIndex(uint2 tilePos, uint tileCountX){
    return tilePos.y * tileCountX + tilePos.x;
}

uint GetTileOffset(uint tileIndex){
    return tileIndex * TILE_SIZE;
}

float PointToPlaneDistance(float3 normal, float d, float3 _point){
    return dot(normal, _point) + d;
}

bool ShpereInFrustum(float4 frustum[6], float3 center, float radius){
    bool inSide = true;
    for(uint i = 0; i < 6; ++i){
        float3 normal = normalize(frustum[i].xyz);
        float b = PointToPlaneDistance(normal, frustum[i].w, center);
        // normal point out of frustum, so it should be negative
        if(b < -radius){
            inSide = false;
            break;
        }
    }
    return inSide;
}

bool PlaneConeIntersect(float3 normal, float d, float3 coneDir, float ){

}

void FillLightGrid(uint2 globalThreadID, uint2 groupID, uint2 threadID, uint threadIndex, uint groupSize){
    float depth = -1.0;
    if(globalThreadID.x >= viewport.x || globalThreadID.y >= viewport.y){
        // out of bound
    }else{
        depth = depthTexture[globalThreadID];
    }

    if(threadID == 0){
        minDepthUInt = 0xffffffff;
        maxDepthUInt = 0;
        tileLightCountSphere = 0;
        tileLightCountCone = 0;
        tileLightCountConeShadowed = 0;
        tileLightBitMask = 0;
    }
    GroupMemoryBarrierWithGroupSync();

    if(depth != -1.0){
        uint depthUint = asuint(depth);
        InterlockedMin(minDepthUInt, depthUint);
        InterlockedMax(maxDepthUInt, depthUint);
    }
    GroupMemoryBarrierWithGroupSync();

    float tileMinDepth = (rcp(asfloat(minDepthUInt)) - 1.0) * rcpZMagic;
    float tileMaxDepth = (rcp(asfloat(maxDepthUInt)) - 1.0) * rcpZMagic;
    float tileDepthRange = tileMaxDepth - tileMinDepth;
    tileDepthRange = max(tileDepthRange, FLT_MIN);
    float invTileDepthRange = rcp(tileDepthRange);

    float2 tileScale = float2(viewport) * invTileDim;
    float3 tileBias = float3(-2.0 * float(groupID.x) + tileScale.x - 1.0, -2.0 * float(groupID.y) + tileScale.y - 1.0, -tileMinDepth * invTileDepthRange);
    float4x4 proj2Tile = float4x4(
        tileScale.x, 0, 0, tileBias.x, // row 1
        0, -tileScale.y, 0, tileBias.y,
        0, 0, invTileDepthRange, tileBias.z,
        0, 0, 0, 1);
    float4x4 tileMVP = mul(proj2Tile, viewProjMatrix);

    // http://www8.cs.umu.se/kurser/5DV051/HT12/lab/plane_extraction.pdf
    // construct frustum from vp matrix, it should be in view space plane
    float4 frustumPlanes[6];
    frustumPlanes[0] = tileMVP[3] + tileMVP[0];
    frustumPlanes[1] = tileMVP[3] - tileMVP[0];
    frustumPlanes[2] = tileMVP[3] + tileMVP[1];
    frustumPlanes[3] = tileMVP[3] - tileMVP[1];
    frustumPlanes[4] = tileMVP[3];
    frustumPlanes[5] = tileMVP[3] - tileMVP[2];
    for (int n = 0; n < 6; n++)
    {
        frustumPlanes[n] *= rsqrt(dot(frustumPlanes[n].xyz, frustumPlanes[n].xyz));
    }

    uint tileIndex = GetTileIndex(groupID, tileCountX);
    uint tileOffset = GetTileOffset(tileIndex);

    // per thread pre lightdata to find set of lights that overlap this tile
    for(uint lightIndex = threadID; lightIndex < MAX_LIGHTS; lightIndex += groupSize){

    }
}
