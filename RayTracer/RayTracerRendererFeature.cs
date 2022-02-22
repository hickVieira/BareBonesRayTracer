using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

namespace RayTracer
{
    public class RayTracerRendererFeature : ScriptableRendererFeature
    {
        public RayTracerRenderPass.BlitToCameraSettings _settings;
        RayTracerRenderPass _rayTracerRenderPass;

        public override void Create()
        {
            _rayTracerRenderPass = new RayTracerRenderPass(_settings);
        }

        public override void AddRenderPasses(ScriptableRenderer renderer, ref RenderingData renderingData)
        {
            renderer.EnqueuePass(_rayTracerRenderPass);
        }

        public class RayTracerRenderPass : ScriptableRenderPass
        {
            [System.Serializable]
            public class BlitToCameraSettings
            {
                public string rtName = "_RayTracerRenderPass";
                public RenderPassEvent renderPassEvent;
                [Range(0.01f, 1)] public float screenSizeFactor = 0.5f;
                public FilterMode filterMode = FilterMode.Trilinear;
                public UnityEngine.Experimental.Rendering.GraphicsFormat format = UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8A8_SRGB;
                public Material RayTracerMaterial;
            }
            BlitToCameraSettings _settings;

            RenderTargetIdentifier _rtID0;
            int _rtNameID0;

            public RayTracerRenderPass(BlitToCameraSettings settings)
            {
                renderPassEvent = settings.renderPassEvent;
                _settings = settings;
            }

            public override void Configure(CommandBuffer cmd, RenderTextureDescriptor cameraTextureDescriptor)
            {
                int width = (int)(cameraTextureDescriptor.width * _settings.screenSizeFactor);
                int height = (int)(cameraTextureDescriptor.height * _settings.screenSizeFactor);

                _rtNameID0 = Shader.PropertyToID(_settings.rtName);
                cmd.GetTemporaryRT(_rtNameID0, width, height, 0, _settings.filterMode, _settings.format);

                _rtID0 = new RenderTargetIdentifier(_rtNameID0);

                ConfigureTarget(_rtID0);
            }

            public override void Execute(ScriptableRenderContext context, ref RenderingData renderingData)
            {
                CommandBuffer cmd = CommandBufferPool.Get();

                Blit(cmd, _rtID0, _rtID0, _settings.RayTracerMaterial, 0);
                Blit(cmd, _rtID0, renderingData.cameraData.renderer.cameraColorTarget);

                context.ExecuteCommandBuffer(cmd);
                CommandBufferPool.Release(cmd);
            }

            public override void FrameCleanup(CommandBuffer cmd)
            {
                cmd.ReleaseTemporaryRT(_rtNameID0);
            }
        }
    }
}