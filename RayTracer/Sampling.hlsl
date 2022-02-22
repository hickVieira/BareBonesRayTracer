#ifndef HICKV_SAMPLING
    #define HICKV_SAMPLING

    #include "Packages/com.unity.render-pipelines.core/ShaderLibrary/Packing.hlsl"

    SAMPLER(SamplerState_Linear_Repeat);
    SAMPLER(SamplerState_Linear_Clamp);

    #define MAIN_SAMPLERSTATE_REPEAT SamplerState_Linear_Repeat
    #define MAIN_SAMPLERSTATE_CLAMP SamplerState_Linear_Clamp

    inline float3 UnpackNormalSample(float4 normalSample, float normalScale)
    {
        #if BUMP_SCALE_NOT_SUPPORTED
            return UnpackNormal(normalSample);
        #else
            return UnpackNormalScale(normalSample, normalScale);
        #endif
    }

    inline half4 SampleTriPlanarTexture2D(Texture2D textureToSample, SamplerState ss, float3 blend, float2 xUV, float2 yUV, float2 zUV)
    {
        half4 texSampleX = SAMPLE_TEXTURE2D(textureToSample, ss, xUV);
        half4 texSampleY = SAMPLE_TEXTURE2D(textureToSample, ss, yUV);
        half4 texSampleZ = SAMPLE_TEXTURE2D(textureToSample, ss, zUV);

        return texSampleX * blend.x + texSampleY * blend.y + texSampleZ * blend.z;
    }

    inline half4 SampleBiPlanarTexture2D_XZ(Texture2D textureToSample, SamplerState ss, float2 blend, float2 xUV, float2 zUV)
    {
        half4 texSampleX = SAMPLE_TEXTURE2D(textureToSample, ss, xUV);
        half4 texSampleZ = SAMPLE_TEXTURE2D(textureToSample, ss, zUV);

        return texSampleX * blend.x + texSampleZ * blend.y;
    }

    inline float3 SampleUnpackTriPlanarNormalMap(Texture2D normalMap, SamplerState ss, float3 blend, float2 xUV, float2 yUV, float2 zUV, float scale)
    {
        float3 tNormalX = UnpackNormalSample(SAMPLE_TEXTURE2D(normalMap, ss, xUV), scale);
        float3 tNormalY = UnpackNormalSample(SAMPLE_TEXTURE2D(normalMap, ss, yUV), scale);
        float3 tNormalZ = UnpackNormalSample(SAMPLE_TEXTURE2D(normalMap, ss, zUV), scale);

        // Swizzle tangemt normals into world space and zero out "z"
        float3 normalX = float3(0.0, tNormalX.yx);
        float3 normalY = float3(tNormalY.x, 0.0, tNormalY.y);
        float3 normalZ = float3(tNormalZ.xy, 0.0);

        // Triblend normals and add to world normal
        return normalX.xyz * blend.x + normalY.xyz * blend.y + normalZ.xyz * blend.z;
    }

    inline float3 SampleUnpackBiPlanarNormalMap_XZ(Texture2D normalMap, SamplerState ss, float2 blend, float2 xUV, float2 zUV, float scale)
    {
        float3 tNormalX = UnpackNormalSample(SAMPLE_TEXTURE2D(normalMap, ss, xUV), scale);
        float3 tNormalZ = UnpackNormalSample(SAMPLE_TEXTURE2D(normalMap, ss, zUV), scale);

        // Swizzle tangemt normals into world space and zero out "z"
        float3 normalX = float3(0.0, tNormalX.yx);
        float3 normalZ = float3(tNormalZ.xy, 0.0);

        // Triblend normals and add to world normal
        return normalX.xyz * blend.x + normalZ.xyz * blend.y;
    }
#endif