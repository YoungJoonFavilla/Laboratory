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
        [Header("Occlusion")]
        public RenderPassEvent unitDepthPassEvent = RenderPassEvent.BeforeRenderingTransparents;
        public RenderPassEvent wallStencilPassEvent = RenderPassEvent.BeforeRenderingTransparents;
        public RenderPassEvent unitOccludedPassEvent = RenderPassEvent.AfterRenderingTransparents;
        public LayerMask unitLayer = -1;
        public LayerMask wallLayer = -1;

        [Header("Outline")]
        public bool enableOutline = true;
        public RenderPassEvent outlinePassEvent = RenderPassEvent.AfterRenderingTransparents;
        public Color outlineColor = new Color(1f, 0.92f, 0.016f, 1f);
        [Range(1, 10)] public int outlineWidth = 3;
    }

    // 패스 간 데이터 공유를 위한 ContextItem
    public class OcclusionFrameData : ContextItem
    {
        public TextureHandle unitDepthTexture;
        public TextureHandle silhouetteMaskTexture;

        public override void Reset()
        {
            unitDepthTexture = TextureHandle.nullHandle;
            silhouetteMaskTexture = TextureHandle.nullHandle;
        }
    }

    public OcclusionSettings settings = new OcclusionSettings();

    private UnitDepthPass _unitDepthPass;
    private WallStencilPass _wallStencilPass;
    private UnitOccludedPass _unitOccludedPass;
    private OutlineSilhouettePass _outlineSilhouettePass;
    private OutlineExpandPass _outlineExpandPass;

    public override void Create()
    {
        _unitDepthPass = new UnitDepthPass(settings);
        _unitDepthPass.renderPassEvent = settings.unitDepthPassEvent;

        _wallStencilPass = new WallStencilPass(settings);
        _wallStencilPass.renderPassEvent = settings.wallStencilPassEvent;

        _unitOccludedPass = new UnitOccludedPass(settings);
        _unitOccludedPass.renderPassEvent = settings.unitOccludedPassEvent;

        _outlineSilhouettePass = new OutlineSilhouettePass(settings);
        _outlineSilhouettePass.renderPassEvent = settings.outlinePassEvent;

        _outlineExpandPass = new OutlineExpandPass(settings);
        _outlineExpandPass.renderPassEvent = settings.outlinePassEvent;
    }

    public override void AddRenderPasses(ScriptableRenderer renderer, ref RenderingData renderingData)
    {
        if (renderingData.cameraData.cameraType != CameraType.Game)
            return;

        renderer.EnqueuePass(_unitDepthPass);
        renderer.EnqueuePass(_wallStencilPass);
        renderer.EnqueuePass(_unitOccludedPass);

        if (settings.enableOutline)
        {
            renderer.EnqueuePass(_outlineSilhouettePass);
            renderer.EnqueuePass(_outlineExpandPass);
        }
    }

    // ============================================
    // Pass 1: 유닛 깊이 쓰기
    // ============================================
    class UnitDepthPass : ScriptableRenderPass
    {
        private OcclusionSettings _settings;
        private List<ShaderTagId> _shaderTagIds = new List<ShaderTagId> { new ShaderTagId("UnitDepthPass") };

        class PassData
        {
            public RendererListHandle rendererListHandle;
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

            var targetDesc = cameraData.cameraTargetDescriptor;
            var textureDesc = new TextureDesc(targetDesc.width, targetDesc.height)
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

                if (!passData.rendererListHandle.IsValid())
                    return;

                builder.UseRendererList(passData.rendererListHandle);
                builder.SetRenderAttachment(unitDepthTex, 0);

                builder.SetRenderFunc(static (PassData data, RasterGraphContext context) =>
                {
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

    // ============================================
    // Pass 4: 아웃라인 실루엣 마스크
    // ============================================
    class OutlineSilhouettePass : ScriptableRenderPass
    {
        private OcclusionSettings _settings;
        private List<ShaderTagId> _shaderTagIds = new List<ShaderTagId> { new ShaderTagId("OutlineSilhouettePass") };

        class PassData
        {
            public RendererListHandle rendererListHandle;
        }

        public OutlineSilhouettePass(OcclusionSettings settings)
        {
            _settings = settings;
        }

        public override void RecordRenderGraph(RenderGraph renderGraph, ContextContainer frameData)
        {
            var cameraData = frameData.Get<UniversalCameraData>();
            var renderingData = frameData.Get<UniversalRenderingData>();
            var lightData = frameData.Get<UniversalLightData>();
            var cullResults = renderingData.cullResults;

            var targetDesc = cameraData.cameraTargetDescriptor;
            var textureDesc = new TextureDesc(targetDesc.width, targetDesc.height)
            {
                colorFormat = UnityEngine.Experimental.Rendering.GraphicsFormat.R8_UNorm,
                clearBuffer = true,
                clearColor = Color.clear,
                name = "_SilhouetteMaskTex"
            };

            TextureHandle silhouetteMaskTex = renderGraph.CreateTexture(textureDesc);

            // OcclusionFrameData에 저장 (없으면 생성)
            OcclusionFrameData occlusionData;
            if (frameData.Contains<OcclusionFrameData>())
                occlusionData = frameData.Get<OcclusionFrameData>();
            else
                occlusionData = frameData.Create<OcclusionFrameData>();
            occlusionData.silhouetteMaskTexture = silhouetteMaskTex;

            using (var builder = renderGraph.AddRasterRenderPass<PassData>("OutlineSilhouettePass", out var passData))
            {
                // 렌더러 리스트 생성 (선택된 유닛만 실제로 렌더링됨 - 셰이더에서 체크)
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
                builder.SetRenderAttachment(silhouetteMaskTex, 0);

                builder.SetRenderFunc(static (PassData data, RasterGraphContext context) =>
                {
                    context.cmd.DrawRendererList(data.rendererListHandle);
                });
            }
        }
    }

    // ============================================
    // Pass 5: 아웃라인 확장 및 컴포지트
    // ============================================
    class OutlineExpandPass : ScriptableRenderPass
    {
        private OcclusionSettings _settings;
        private Material _outlineExpandMaterial;

        private static readonly int SilhouetteMaskTexId = Shader.PropertyToID("_MainTex");
        private static readonly int OutlineColorId = Shader.PropertyToID("_OutlineColor");
        private static readonly int OutlineWidthId = Shader.PropertyToID("_OutlineWidth");

        class PassData
        {
            public TextureHandle silhouetteMaskTexture;
            public Material material;
            public Color outlineColor;
            public int outlineWidth;
        }

        public OutlineExpandPass(OcclusionSettings settings)
        {
            _settings = settings;
        }

        public override void RecordRenderGraph(RenderGraph renderGraph, ContextContainer frameData)
        {
            var resourceData = frameData.Get<UniversalResourceData>();

            // OcclusionFrameData 체크
            if (!frameData.Contains<OcclusionFrameData>())
                return;

            var occlusionData = frameData.Get<OcclusionFrameData>();
            if (!occlusionData.silhouetteMaskTexture.IsValid())
                return;

            // 머티리얼 로드 (최초 1회)
            if (_outlineExpandMaterial == null)
            {
                var shader = Shader.Find("Hidden/OutlineExpand");
                if (shader == null)
                {
                    Debug.LogError("OutlineExpand shader not found!");
                    return;
                }
                _outlineExpandMaterial = new Material(shader);
            }

            using (var builder = renderGraph.AddRasterRenderPass<PassData>("OutlineExpandPass", out var passData))
            {
                passData.silhouetteMaskTexture = occlusionData.silhouetteMaskTexture;
                passData.material = _outlineExpandMaterial;
                passData.outlineColor = _settings.outlineColor;
                passData.outlineWidth = _settings.outlineWidth;

                builder.UseTexture(passData.silhouetteMaskTexture);
                builder.SetRenderAttachment(resourceData.activeColorTexture, 0);

                builder.SetRenderFunc(static (PassData data, RasterGraphContext context) =>
                {
                    data.material.SetTexture(SilhouetteMaskTexId, data.silhouetteMaskTexture);
                    data.material.SetColor(OutlineColorId, data.outlineColor);
                    data.material.SetFloat(OutlineWidthId, data.outlineWidth);

                    // Fullscreen blit
                    context.cmd.DrawProcedural(Matrix4x4.identity, data.material, 0, MeshTopology.Triangles, 3);
                });
            }
        }
    }
}
