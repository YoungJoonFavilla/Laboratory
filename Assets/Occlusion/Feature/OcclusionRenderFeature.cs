using UnityEngine;
using UnityEngine.Rendering;
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

    public OcclusionSettings settings = new OcclusionSettings();

    private UnitDepthPass _unitDepthPass;
    private WallStencilPass _wallStencilPass;
    private UnitOccludedPass _unitOccludedPass;
    private RTHandle _unitDepthRT;

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

        // 유닛 위치 RT 생성/갱신 (XY 좌표 저장)
        var desc = renderingData.cameraData.cameraTargetDescriptor;
        desc.colorFormat = RenderTextureFormat.RGFloat;
        desc.depthBufferBits = 0;

        RenderingUtils.ReAllocateIfNeeded(ref _unitDepthRT, desc, FilterMode.Point, TextureWrapMode.Clamp, name: "_UnitDepthTex");

        _unitDepthPass.Setup(_unitDepthRT);
        _wallStencilPass.Setup(_unitDepthRT, renderer);
        _unitOccludedPass.Setup(renderer);

        renderer.EnqueuePass(_unitDepthPass);
        renderer.EnqueuePass(_wallStencilPass);
        renderer.EnqueuePass(_unitOccludedPass);
    }

    protected override void Dispose(bool disposing)
    {
        _unitDepthRT?.Release();
    }

    // ============================================
    // Pass 1: 유닛 깊이 쓰기
    // ============================================
    class UnitDepthPass : ScriptableRenderPass
    {
        private OcclusionSettings _settings;
        private RTHandle _depthRT;
        private FilteringSettings _filteringSettings;
        private ShaderTagId _shaderTagId = new ShaderTagId("UnitDepthPass");

        private static readonly int DepthDirId = Shader.PropertyToID("_OcclusionDepthDir");
        private static readonly int MinDepthId = Shader.PropertyToID("_OcclusionMinDepth");
        private static readonly int MaxDepthId = Shader.PropertyToID("_OcclusionMaxDepth");

        public UnitDepthPass(OcclusionSettings settings)
        {
            _settings = settings;
            _filteringSettings = new FilteringSettings(RenderQueueRange.transparent, settings.unitLayer);
        }

        public void Setup(RTHandle depthRT)
        {
            _depthRT = depthRT;
        }

        public override void OnCameraSetup(CommandBuffer cmd, ref RenderingData renderingData)
        {
            ConfigureTarget(_depthRT);
            ConfigureClear(ClearFlag.Color, Color.clear);
        }

        public override void Execute(ScriptableRenderContext context, ref RenderingData renderingData)
        {
            CommandBuffer cmd = CommandBufferPool.Get("UnitDepthPass");

            // 글로벌 셰이더 변수 설정
            float rad = _settings.projectionAngle * Mathf.Deg2Rad;
            Vector4 depthDir = new Vector4(Mathf.Cos(rad), Mathf.Sin(rad), 0, 0);

            cmd.SetGlobalVector(DepthDirId, depthDir);
            cmd.SetGlobalFloat(MinDepthId, _settings.minDepth);
            cmd.SetGlobalFloat(MaxDepthId, _settings.maxDepth);

            context.ExecuteCommandBuffer(cmd);
            cmd.Clear();

            // 유닛 렌더링
            var drawingSettings = CreateDrawingSettings(_shaderTagId, ref renderingData, SortingCriteria.CommonTransparent);
            context.DrawRenderers(renderingData.cullResults, ref drawingSettings, ref _filteringSettings);

            context.ExecuteCommandBuffer(cmd);
            CommandBufferPool.Release(cmd);
        }
    }

    // ============================================
    // Pass 2: 벽 스텐실 마킹
    // ============================================
    class WallStencilPass : ScriptableRenderPass
    {
        private OcclusionSettings _settings;
        private RTHandle _unitDepthRT;
        private ScriptableRenderer _renderer;
        private FilteringSettings _filteringSettings;
        private ShaderTagId _shaderTagId = new ShaderTagId("WallStencilPass");

        private static readonly int UnitDepthTexId = Shader.PropertyToID("_UnitDepthTex");

        public WallStencilPass(OcclusionSettings settings)
        {
            _settings = settings;
            _filteringSettings = new FilteringSettings(RenderQueueRange.all, settings.wallLayer);
        }

        public void Setup(RTHandle unitDepthRT, ScriptableRenderer renderer)
        {
            _unitDepthRT = unitDepthRT;
            _renderer = renderer;
        }

        public override void OnCameraSetup(CommandBuffer cmd, ref RenderingData renderingData)
        {
            // 카메라의 컬러+스텐실 버퍼로 타겟 설정
            ConfigureTarget(_renderer.cameraColorTargetHandle, _renderer.cameraDepthTargetHandle);
        }

        public override void Execute(ScriptableRenderContext context, ref RenderingData renderingData)
        {
            CommandBuffer cmd = CommandBufferPool.Get("WallStencilPass");

            // 유닛 깊이 텍스처를 글로벌로 설정
            cmd.SetGlobalTexture(UnitDepthTexId, _unitDepthRT);

            context.ExecuteCommandBuffer(cmd);
            cmd.Clear();

            // 벽 렌더링 (스텐실 마킹)
            var drawingSettings = CreateDrawingSettings(_shaderTagId, ref renderingData, SortingCriteria.CommonOpaque);
            context.DrawRenderers(renderingData.cullResults, ref drawingSettings, ref _filteringSettings);

            context.ExecuteCommandBuffer(cmd);
            CommandBufferPool.Release(cmd);
        }
    }

    // ============================================
    // Pass 3: 가려진 유닛 렌더링
    // ============================================
    class UnitOccludedPass : ScriptableRenderPass
    {
        private OcclusionSettings _settings;
        private ScriptableRenderer _renderer;
        private FilteringSettings _filteringSettings;
        private ShaderTagId _shaderTagId = new ShaderTagId("UnitOccludedPass");

        public UnitOccludedPass(OcclusionSettings settings)
        {
            _settings = settings;
            _filteringSettings = new FilteringSettings(RenderQueueRange.transparent, settings.unitLayer);
        }

        public void Setup(ScriptableRenderer renderer)
        {
            _renderer = renderer;
        }

        public override void OnCameraSetup(CommandBuffer cmd, ref RenderingData renderingData)
        {
            ConfigureTarget(_renderer.cameraColorTargetHandle, _renderer.cameraDepthTargetHandle);
        }

        public override void Execute(ScriptableRenderContext context, ref RenderingData renderingData)
        {
            CommandBuffer cmd = CommandBufferPool.Get("UnitOccludedPass");
            context.ExecuteCommandBuffer(cmd);
            cmd.Clear();

            // 가려진 유닛 렌더링
            var drawingSettings = CreateDrawingSettings(_shaderTagId, ref renderingData, SortingCriteria.CommonTransparent);
            context.DrawRenderers(renderingData.cullResults, ref drawingSettings, ref _filteringSettings);

            context.ExecuteCommandBuffer(cmd);
            CommandBufferPool.Release(cmd);
        }
    }
}
