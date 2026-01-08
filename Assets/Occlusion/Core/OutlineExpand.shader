Shader "Hidden/OutlineExpand"
{
    Properties
    {
        _MainTex("Silhouette Mask", 2D) = "black" {}
        _OutlineColor("Outline Color", Color) = (1, 0.92, 0.016, 1)
        _OutlineWidth("Outline Width", Range(1, 10)) = 3
    }

    SubShader
    {
        Tags
        {
            "RenderPipeline"="UniversalPipeline"
        }

        Pass
        {
            Name "OutlineExpand"

            Blend SrcAlpha OneMinusSrcAlpha
            ZWrite Off
            ZTest Always
            Cull Off

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"

            struct Varyings
            {
                float4 positionCS : SV_POSITION;
                float2 uv : TEXCOORD0;
            };

            TEXTURE2D(_MainTex);
            SAMPLER(sampler_MainTex);
            float4 _MainTex_TexelSize;

            float4 _OutlineColor;
            float _OutlineWidth;

            // Fullscreen triangle vertex shader
            Varyings vert(uint vertexID : SV_VertexID)
            {
                Varyings o;
                // Generate fullscreen triangle
                o.positionCS = float4(
                    vertexID <= 1 ? -1.0 : 3.0,
                    vertexID == 1 ? 3.0 : -1.0,
                    0.0, 1.0
                );
                o.uv = float2(
                    vertexID <= 1 ? 0.0 : 2.0,
                    vertexID == 1 ? 2.0 : 0.0
                );
                // Flip Y for DirectX
                #if UNITY_UV_STARTS_AT_TOP
                o.uv.y = 1.0 - o.uv.y;
                #endif
                return o;
            }

            half4 frag(Varyings i) : SV_Target
            {
                // 현재 픽셀의 마스크 값
                float center = SAMPLE_TEXTURE2D(_MainTex, sampler_MainTex, i.uv).r;

                // 실루엣 내부는 투명 처리 (아웃라인만 그림)
                if (center > 0.5)
                    return half4(0, 0, 0, 0);

                // 고정 샘플링 (최대 10픽셀 너비 지원)
                // 8방향 + 거리별 샘플링으로 최적화
                float maxNeighbor = 0;

                // 4방향 (상하좌우)
                [unroll]
                for (int dist = 1; dist <= 10; dist++)
                {
                    if (dist > (int)_OutlineWidth)
                        break;

                    float2 offset = _MainTex_TexelSize.xy * dist;

                    maxNeighbor = max(maxNeighbor, SAMPLE_TEXTURE2D(_MainTex, sampler_MainTex, i.uv + float2(offset.x, 0)).r);
                    maxNeighbor = max(maxNeighbor, SAMPLE_TEXTURE2D(_MainTex, sampler_MainTex, i.uv + float2(-offset.x, 0)).r);
                    maxNeighbor = max(maxNeighbor, SAMPLE_TEXTURE2D(_MainTex, sampler_MainTex, i.uv + float2(0, offset.y)).r);
                    maxNeighbor = max(maxNeighbor, SAMPLE_TEXTURE2D(_MainTex, sampler_MainTex, i.uv + float2(0, -offset.y)).r);

                    // 대각선
                    float2 diagOffset = offset * 0.707;
                    maxNeighbor = max(maxNeighbor, SAMPLE_TEXTURE2D(_MainTex, sampler_MainTex, i.uv + float2(diagOffset.x, diagOffset.y)).r);
                    maxNeighbor = max(maxNeighbor, SAMPLE_TEXTURE2D(_MainTex, sampler_MainTex, i.uv + float2(-diagOffset.x, diagOffset.y)).r);
                    maxNeighbor = max(maxNeighbor, SAMPLE_TEXTURE2D(_MainTex, sampler_MainTex, i.uv + float2(diagOffset.x, -diagOffset.y)).r);
                    maxNeighbor = max(maxNeighbor, SAMPLE_TEXTURE2D(_MainTex, sampler_MainTex, i.uv + float2(-diagOffset.x, -diagOffset.y)).r);

                    if (maxNeighbor > 0.5)
                        break;
                }

                // 주변에 실루엣이 있으면 아웃라인
                if (maxNeighbor > 0.5)
                    return _OutlineColor;

                return half4(0, 0, 0, 0);
            }
            ENDHLSL
        }
    }
}
