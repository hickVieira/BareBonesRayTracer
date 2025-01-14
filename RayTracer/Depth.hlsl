#ifndef HICKV_DEPTH
    #define HICKV_DEPTH

    #include "Packages/com.unity.render-pipelines.core/ShaderLibrary/Common.hlsl"
    #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/DeclareDepthTexture.hlsl"
    #include "Projection.hlsl"

    float4 _CameraDepthTexture_TexelSize;

    // Get a raw depth from the depth buffer.
    float SampleRawDepth(float2 uv)
    {
        float z = SampleSceneDepth(uv);
        #if defined(UNITY_REVERSED_Z)
            z = 1 - z;
        #endif
        return z;
    }

    float RawToLinearDepth(float rawDepth)
    {
        #if defined(_ORTHOGRAPHIC)
            #if UNITY_REVERSED_Z
                return ((_ProjectionParams.z - _ProjectionParams.y) * (1.0 - rawDepth) + _ProjectionParams.y);
            #else
                return ((_ProjectionParams.z - _ProjectionParams.y) * (rawDepth) + _ProjectionParams.y);
            #endif
        #else
            return LinearEyeDepth(rawDepth, _ZBufferParams);
        #endif
    }

    float LinearToDepth(float linearDepth)
    {
        return (1.0 - _ZBufferParams.w * linearDepth) / (linearDepth * _ZBufferParams.z);
    }

    float3 NormalFromDepth_4Tap(float2 uv, float depthRawC, float3 posCenter)
    {
        float2 uvU = uv + float2(0, 1) * _CameraDepthTexture_TexelSize.y; // up right
        float2 uvD = uv + float2(0, -1) * _CameraDepthTexture_TexelSize.y; // down left
        float2 uvL = uv + float2(-1, 0) * _CameraDepthTexture_TexelSize.x; // up left
        float2 uvR = uv + float2(1, 0) * _CameraDepthTexture_TexelSize.x; // down right

        float depthRawU = SampleRawDepth(uvU);
        float depthRawD = SampleRawDepth(uvD);
        float depthRawL = SampleRawDepth(uvL);
        float depthRawR = SampleRawDepth(uvR);

        float3 PC = posCenter;
        float3 P1;
        float3 P2;

        // compare samples to center-depth in counter-clockwise order and make a "triangle"
        // https://wickedengine.net/2019/09/22/improved-normal-reconstruction-from-depth/

        float distU = distance(depthRawU, depthRawC);
        float distD = distance(depthRawD, depthRawC);
        float distL = distance(depthRawL, depthRawC);
        float distR = distance(depthRawR, depthRawC);
        
        // triangle 0 = P0: center, P1: right, P2: up
        // triangle 2 = P0: center, P1: up, P2: left
        if(distU <= distD)
        {
            if(distR <= distL)
            {
                P1 = ProjectNDCtoWS(uvR, depthRawR);
                P2 = ProjectNDCtoWS(uvU, depthRawU);
            }
            else
            {
                P1 = ProjectNDCtoWS(uvU, depthRawU);
                P2 = ProjectNDCtoWS(uvL, depthRawL);
            }
        }
        // triangle 1 = P0: center, P1: down, P2: right
        // triangle 3 = P0: center, P1: left, P2: down
        else
        {
            if(distR <= distL)
            {
                P1 = ProjectNDCtoWS(uvD, depthRawD);
                P2 = ProjectNDCtoWS(uvR, depthRawR);
            }
            else
            {
                P1 = ProjectNDCtoWS(uvL, depthRawL);
                P2 = ProjectNDCtoWS(uvD, depthRawD);
            }
        }

        return normalize(cross(P2 - PC, P1 - PC));
    }
    
#endif
