Shader "hickv/RayTracer"
{
    Properties
    {
        _BlueNoiseRGBA("BlueNoiseRGBA", 2D) = "black" {}
        _SunSize("SunSize", Range(0,0.5)) = 0.01
        _SunTraceDistance("SunTraceDistance", Range(0,100)) = 1
        _SkyTraceDistance("SkyTraceDistance", Range(0,100)) = 1
        [IntRange] _SunSamples("SunSamples", Range(0,256)) = 1
        [IntRange] _SkySamples("SkySamples", Range(0,256)) = 1
    }
    SubShader
    {
        Blend one zero
        ZWrite off

        // raytrace pass
        Pass
        {
            HLSLPROGRAM
            #pragma target 4.5

            #pragma vertex vert
            #pragma fragment frag

            #pragma multi_compile_fragment _ _GBUFFER_NORMALS_OCT

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/DeclareNormalsTexture.hlsl"
            #include "BVH.hlsl"
            #include "Constants.hlsl"
            #include "Depth.hlsl"
            #include "Projection.hlsl"
            #include "Noise.hlsl"

            SAMPLER(SamplerState_Linear_Repeat);
            TEXTURE2D(_BlueNoiseRGBA);
            float _SunSize;
            float _SunTraceDistance;
            float _SkyTraceDistance;
            int _SunSamples;
            int _SkySamples;

            // #define CHECK_HIT_TRIANGLE_ALL 1

            struct Attributes
            {
                float2 uv           : TEXCOORD0;
            };

            struct Varyings
            {
                float2 uv               : TEXCOORD0;
                float4 positionCS       : SV_POSITION;
            };

            bool RayTrace(in float3 positionWS, in float3 directionWS, out float outRayScale, out float3 outNormal)
            {
                float heat;

                #ifdef CHECK_HIT_TRIANGLE_ALL
                    if (LineTriangleIntersectionAll(positionWS, directionWS, outRayScale, outNormal))
                #else
                    if (TraverseBVH(positionWS, directionWS, outRayScale, outNormal, heat))
                #endif
                return true;
                return false;
            }

            Varyings vert(Attributes IN)
            {
                Varyings OUT;

                OUT.uv = IN.uv;

                float2 cp = float2(IN.uv * 2 - 1);
                OUT.positionCS = float4(cp.x, -cp.y, 1, 1);

                return OUT;
            }

            float4 frag(Varyings IN) : SV_Target
            {
                float depthRaw = SampleRawDepth(IN.uv);

                if (depthRaw >= 1) return 0;

                const float surfaceOffset = 0.00025;

                float3 normalWS = SampleSceneNormals(IN.uv);
                float3 positionWS = ProjectNDCtoWS(IN.uv, depthRaw);
                float surfaceDistance = distance(positionWS, _WorldSpaceCameraPos);
                positionWS = positionWS + normalWS * surfaceOffset * surfaceDistance;

                // float4 blueNoise = SAMPLE_TEXTURE2D(_BlueNoiseRGBA, SamplerState_Linear_Repeat, IN.uv + fmod(i * 1000, 100));

                float outRayScale;
                float3 outNormal;
                float3 sunLight = 0;
                float3 skyLight = 0;
                
                // Sun
                float sunLightNDotL = dot(_MainLightPosition.xyz, normalWS);
                if (sunLightNDotL > 0)
                {
                    UNITY_LOOP
                    for (int i = 0; i < _SunSamples; i++)
                    {
                        // float noise0 = blueNoise[(i + 0) % 4];
                        // float noise1 = blueNoise[(i + 1) % 4];
                        // float noise2 = blueNoise[(i + 2) % 4];
                        // float3 noiseVector = normalize(float3(noise0 * 2 - 1, noise1 * 2 - 1, noise2 * 2 - 1));

                        float noise0 = random2d(IN.uv + positionWS.x + i) * 2 - 1;
                        float noise1 = random2d(IN.uv + positionWS.y + i) * 2 - 1;
                        float noise2 = random2d(IN.uv + positionWS.z + i) * 2 - 1;
                        float3 noiseVector = float3(noise0, noise1, noise2);

                        float3 sunVector = normalize(_MainLightPosition.xyz + noiseVector * _SunSize) * _SunTraceDistance;
                        sunLight += _MainLightColor * sunLightNDotL * (1 - RayTrace(positionWS, sunVector, outRayScale, outNormal));
                    }
                    sunLight /= _SunSamples;
                }

                // Sky
                UNITY_LOOP
                for (int i = 0; i < _SkySamples; i++)
                {
                    // float noise1 = blueNoise[4 - (100 * (_Time.y * (i + 1)) % 4)];
                    // float noise2 = blueNoise[4 - (100 * (_Time.y * (i + 2)) % 4)];
                    // float noise3 = blueNoise[4 - (100 * (_Time.y * (i + 3)) % 4)];
                    // float3 noiseVector = normalize(float3(noise1 * 2 - 1, noise2 * 2 - 1, noise3 * 2 - 1));

                    float noise0 = random2d(IN.uv + positionWS.x + i) * 2 - 1;
                    float noise1 = random2d(IN.uv + positionWS.y + i) * 2 - 1;
                    float noise2 = random2d(IN.uv + positionWS.z + i) * 2 - 1;
                    float3 noiseVector = float3(noise0, noise1, noise2);
                    
                    float3 skyVector = normalize(normalWS + noiseVector) * _SkyTraceDistance;
                    skyLight += 0.25 * float3(0.2,0.5,0.75) * (1 - RayTrace(positionWS, skyVector, outRayScale, outNormal));
                }
                skyLight /= _SkySamples;

                return float4(sunLight + skyLight, 0);
            }
            ENDHLSL
        }
    }
}
