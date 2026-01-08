using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.RenderGraphModule;
using UnityEngine.Rendering.Universal;

public class OcclusionRenderFeature : ScriptableRendererFeature
{
    [System.Serializable]
    public class OcclusionSettings
    {
        public RenderPassEvent unitDepthPassEvent = RenderPassEvent.BeforeRenderingTransparents;
        public RenderPassEvent wallStencilPassEvent = RenderPassEvent.BeforeRenderingTransparents;
        public RenderPassEvent unitOccludedPassEvent = RenderPassEvent.AfterRenderingTransparents;
        public LayerMask unitLayer = -1;
        public LayerMask wallLayer = -1;

        [Header("Depth Settings")]
        public float projectionAngle = OccConstant.DEFAULT_NORMAL_ANGLE;
        public float minDepth = -50f;
        public float maxDepth = 50f;
    }

    // 패스 간 데이터 공유를 위한 ContextItem
    public class OcclusionFrameData : ContextItem
    {
        public TextureHandle unitDepthTexture;

        public override void Reset()
        {
            unitDepthTexture = TextureHandle.nullHandle;
        }
    }

    public OcclusionSettings settings = new OcclusionSettings();

    private UnitDepthPass _unitDepthPass;
    private WallStencilPass _wallStencilPass;
    private UnitOccludedPass _unitOccludedPass;

    public override void Create()
    {
        _unitDepthPass = new UnitDepthPass(settings);
        _unitDepthPass.renderPassEvent = settings.unitDepthPassEvent;

        _wallStencilPass = new WallStencilPass(settings);
        _wallStencilPass.renderPassEvent = settings.wallStencilPassEvent;

        _unitOccludedPass = new UnitOccludedPass(settings);
        _unitOccludedPass.renderPassEvent = settings.unitOccludedPassEvent;
    }

    public override void AddRenderPasses(ScriptableRenderer renderer, ref RenderingData renderingData)
    {
        if (renderingData.cameraData.cameraType != CameraType.Game)
            return;

        renderer.EnqueuePass(_unitDepthPass);
        renderer.EnqueuePass(_wallStencilPass);
        renderer.EnqueuePass(_unitOccludedPass);
    }

    // ============================================
    // Pass 1: 유닛 깊이 쓰기
    // ============================================
    class UnitDepthPass : ScriptableRenderPass
    {
        private OcclusionSettings _settings;
        private List<ShaderTagId> _shaderTagIds = new List<ShaderTagId> { new ShaderTagId("UnitDepthPass") };

        private static readonly int DepthDirId = Shader.PropertyToID("_OcclusionDepthDir");
        private static readonly int MinDepthId = Shader.PropertyToID("_OcclusionMinDepth");
        private static readonly int MaxDepthId = Shader.PropertyToID("_OcclusionMaxDepth");

        class PassData
        {
            public RendererListHandle rendererListHandle;
            public Vector4 depthDir;
            public float minDepth;
            public float maxDepth;
        }

        public UnitDepthPass(OcclusionSettings settings)
        {
            _settings = settings;
        }

        public override void RecordRenderGraph(RenderGraph renderGraph, ContextContainer frameData)
        {
            var cameraData = frameData.Get<UniversalCameraData>();
            var cullResults = frameData.Get<UniversalRenderingData>().cullResults;
            var lightData = frameData.Get<UniversalLightData>();

            // 유닛 위치 RT 생성 (XY 좌표 저장)
            var desc = cameraData.cameraTargetDescriptor;
            desc.colorFormat = RenderTextureFormat.RGFloat;
            desc.depthBufferBits = 0;
            desc.msaaSamples = 1;

            var textureDesc = new TextureDesc(desc.width, desc.height)
            {
                colorFormat = UnityEngine.Experimental.Rendering.GraphicsFormat.R32G32_SFloat,
                clearBuffer = true,
                clearColor = Color.clear,
                name = "_UnitDepthTex"
            };

            TextureHandle unitDepthTex = renderGraph.CreateTexture(textureDesc);

            // OcclusionFrameData에 텍스처 핸들 저장 (다른 패스에서 사용)
            var occlusionData = frameData.Create<OcclusionFrameData>();
            occlusionData.unitDepthTexture = unitDepthTex;

            using (var builder = renderGraph.AddRasterRenderPass<PassData>("UnitDepthPass", out var passData))
            {
                // 렌더러 리스트 생성
                var filterSettings = new FilteringSettings(RenderQueueRange.transparent, _settings.unitLayer);
                var drawSettings = RenderingUtils.CreateDrawingSettings(
                    _shaderTagIds,
                    frameData.Get<UniversalRenderingData>(),
                    cameraData,
                    lightData,
                    SortingCriteria.CommonTransparent
                );

                var param = new RendererListParams(cullResults, drawSettings, filterSettings);
                passData.rendererListHandle = renderGraph.CreateRendererList(param);

                // 셰이더 파라미터 계산
                float rad = _settings.projectionAngle * Mathf.Deg2Rad;
                passData.depthDir = new Vector4(Mathf.Cos(rad), Mathf.Sin(rad), 0, 0);
                passData.minDepth = _settings.minDepth;
                passData.maxDepth = _settings.maxDepth;

                if (!passData.rendererListHandle.IsValid())
                    return;

                builder.UseRendererList(passData.rendererListHandle);
                builder.SetRenderAttachment(unitDepthTex, 0);
                builder.AllowGlobalStateModification(true);

                builder.SetRenderFunc(static (PassData data, RasterGraphContext context) =>
                {
                    // 글로벌 셰이더 변수 설정
                    context.cmd.SetGlobalVector(DepthDirId, data.depthDir);
                    context.cmd.SetGlobalFloat(MinDepthId, data.minDepth);
                    context.cmd.SetGlobalFloat(MaxDepthId, data.maxDepth);

                    // 유닛 렌더링
                    context.cmd.DrawRendererList(data.rendererListHandle);
                });
            }
        }
    }

    // ============================================
    // Pass 2: 벽 스텐실 마킹
    // ============================================
    class WallStencilPass : ScriptableRenderPass
    {
        private OcclusionSettings _settings;
        private List<ShaderTagId> _shaderTagIds = new List<ShaderTagId> { new ShaderTagId("WallStencilPass") };

        private static readonly int UnitDepthTexId = Shader.PropertyToID("_UnitDepthTex");

        class PassData
        {
            public RendererListHandle rendererListHandle;
            public TextureHandle unitDepthTexture;
        }

        public WallStencilPass(OcclusionSettings settings)
        {
            _settings = settings;
        }

        public override void RecordRenderGraph(RenderGraph renderGraph, ContextContainer frameData)
        {
            var resourceData = frameData.Get<UniversalResourceData>();
            var cameraData = frameData.Get<UniversalCameraData>();
            var renderingData = frameData.Get<UniversalRenderingData>();
            var lightData = frameData.Get<UniversalLightData>();
            var cullResults = renderingData.cullResults;

            // 이전 패스에서 생성한 유닛 깊이 텍스처 가져오기
            var occlusionData = frameData.Get<OcclusionFrameData>();
            if (!occlusionData.unitDepthTexture.IsValid())
                return;

            using (var builder = renderGraph.AddRasterRenderPass<PassData>("WallStencilPass", out var passData))
            {
                // 렌더러 리스트 생성
                var filterSettings = new FilteringSettings(RenderQueueRange.all, _settings.wallLayer);
                var drawSettings = RenderingUtils.CreateDrawingSettings(
                    _shaderTagIds,
                    renderingData,
                    cameraData,
                    lightData,
                    SortingCriteria.CommonOpaque
                );

                var param = new RendererListParams(cullResults, drawSettings, filterSettings);
                passData.rendererListHandle = renderGraph.CreateRendererList(param);
                passData.unitDepthTexture = occlusionData.unitDepthTexture;

                if (!passData.rendererListHandle.IsValid())
                    return;

                builder.UseRendererList(passData.rendererListHandle);
                builder.UseTexture(passData.unitDepthTexture);

                // 카메라의 컬러+깊이(스텐실) 버퍼로 타겟 설정
                builder.SetRenderAttachment(resourceData.activeColorTexture, 0);
                builder.SetRenderAttachmentDepth(resourceData.activeDepthTexture, AccessFlags.ReadWrite);
                builder.AllowGlobalStateModification(true);

                builder.SetRenderFunc(static (PassData data, RasterGraphContext context) =>
                {
                    // 유닛 깊이 텍스처를 글로벌로 설정
                    context.cmd.SetGlobalTexture(UnitDepthTexId, data.unitDepthTexture);

                    // 벽 렌더링 (스텐실 마킹)
                    context.cmd.DrawRendererList(data.rendererListHandle);
                });
            }
        }
    }

    // ============================================
    // Pass 3: 가려진 유닛 렌더링
    // ============================================
    class UnitOccludedPass : ScriptableRenderPass
    {
        private OcclusionSettings _settings;
        private List<ShaderTagId> _shaderTagIds = new List<ShaderTagId> { new ShaderTagId("UnitOccludedPass") };

        class PassData
        {
            public RendererListHandle rendererListHandle;
        }

        public UnitOccludedPass(OcclusionSettings settings)
        {
            _settings = settings;
        }

        public override void RecordRenderGraph(RenderGraph renderGraph, ContextContainer frameData)
        {
            var resourceData = frameData.Get<UniversalResourceData>();
            var cameraData = frameData.Get<UniversalCameraData>();
            var renderingData = frameData.Get<UniversalRenderingData>();
            var lightData = frameData.Get<UniversalLightData>();
            var cullResults = renderingData.cullResults;

            using (var builder = renderGraph.AddRasterRenderPass<PassData>("UnitOccludedPass", out var passData))
            {
                // 렌더러 리스트 생성
                var filterSettings = new FilteringSettings(RenderQueueRange.transparent, _settings.unitLayer);
                var drawSettings = RenderingUtils.CreateDrawingSettings(
                    _shaderTagIds,
                    renderingData,
                    cameraData,
                    lightData,
                    SortingCriteria.CommonTransparent
                );

                var param = new RendererListParams(cullResults, drawSettings, filterSettings);
                passData.rendererListHandle = renderGraph.CreateRendererList(param);

                if (!passData.rendererListHandle.IsValid())
                    return;

                builder.UseRendererList(passData.rendererListHandle);

                // 카메라의 컬러+깊이(스텐실) 버퍼로 타겟 설정
                builder.SetRenderAttachment(resourceData.activeColorTexture, 0);
                builder.SetRenderAttachmentDepth(resourceData.activeDepthTexture, AccessFlags.Read);

                builder.SetRenderFunc(static (PassData data, RasterGraphContext context) =>
                {
                    // 가려진 유닛 렌더링 (스텐실=1인 영역에서만)
                    context.cmd.DrawRendererList(data.rendererListHandle);
                });
            }
        }
    }
}
