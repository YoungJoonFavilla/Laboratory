Shader "Custom/WallStencilDepthTex"
{
    Properties
    {
        _MainTex("Texture", 2D) = "white" {}
        _Color("Color", Color) = (1,1,1,1)
        [HideInInspector] _WallBasePos("Wall Base Position (X, Y)", Vector) = (0, 0, 0, 0)
        [HideInInspector] _WallDepthDir("Wall Depth Direction", Vector) = (0, 1, 0, 0)
        _DepthBias("Depth Bias (Anti-Flicker)", Float) = 0.5
        [Toggle] _DebugAlwaysOcclude("Debug: Always Occlude", Float) = 0
        [Toggle] _DebugShowDepth("Debug: Show Depth Compare", Float) = 0
    }

    SubShader
    {
        Tags
        {
            "RenderPipeline"="UniversalPipeline"
            "Queue"="Geometry-1"
            "RenderType"="Transparent"
        }

        // Pass 1: 스텐실 마킹
        Pass
        {
            Name "WallStencil"
            Tags { "LightMode"="WallStencilPass" }

            ZWrite Off
            ZTest Always
            Cull Off
            ColorMask 0

            Stencil
            {
                Ref 1
                Comp Always
                Pass Replace
            }

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"

            struct Attributes
            {
                float4 positionOS : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct Varyings
            {
                float4 positionHCS : SV_POSITION;
                float2 uv : TEXCOORD0;
                float3 worldPos : TEXCOORD1;
                float4 screenPos : TEXCOORD2;
            };

            TEXTURE2D(_MainTex);
            TEXTURE2D(_UnitDepthTex);
            SAMPLER(sampler_MainTex);
            SAMPLER(sampler_UnitDepthTex);

            float4 _WallBasePos;
            float4 _WallDepthDir;
            float _DepthBias;
            float _DebugAlwaysOcclude;

            Varyings vert(Attributes v)
            {
                Varyings o;
                VertexPositionInputs pos = GetVertexPositionInputs(v.positionOS.xyz);
                o.positionHCS = pos.positionCS;
                o.worldPos = pos.positionWS;
                o.uv = v.uv;
                o.screenPos = ComputeScreenPos(o.positionHCS);
                return o;
            }

            half4 frag(Varyings i) : SV_Target
            {
                half4 tex = SAMPLE_TEXTURE2D(_MainTex, sampler_MainTex, i.uv);
                if (tex.a < 0.01)
                    discard;

                // 디버그 모드: 항상 스텐실 마킹
                if (_DebugAlwaysOcclude > 0.5)
                    return 0;

                // 스크린 UV 계산
                float2 screenUV = i.screenPos.xy / i.screenPos.w;

                // 유닛 위치 텍스처 샘플링 (XY 좌표)
                float2 unitPos = SAMPLE_TEXTURE2D(_UnitDepthTex, sampler_UnitDepthTex, screenUV).rg;

                // 유닛이 없는 픽셀 (XY가 0,0이면 유닛 없음)
                if (abs(unitPos.x) < 0.001 && abs(unitPos.y) < 0.001)
                    discard;

                // 벽의 투영 방향으로 유닛 깊이 계산
                float unitDepth = dot(unitPos, _WallDepthDir.xy);

                // 벽 깊이 계산 (벽의 투영 방향 사용)
                float wallDepth = dot(_WallBasePos.xy, _WallDepthDir.xy);

                // 벽이 유닛보다 앞에 있어야 오클루전
                if (wallDepth + _DepthBias >= unitDepth)
                    discard;

                return 0;
            }
            ENDHLSL
        }

        // Pass 2: 텍스처 렌더링 (2D 라이팅 지원)
        Pass
        {
            Name "WallRender"
            Tags { "LightMode"="Universal2D" }

            Blend SrcAlpha OneMinusSrcAlpha
            ZWrite Off
            Cull Off

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #pragma multi_compile USE_SHAPE_LIGHT_TYPE_0 __
            #pragma multi_compile USE_SHAPE_LIGHT_TYPE_1 __
            #pragma multi_compile USE_SHAPE_LIGHT_TYPE_2 __
            #pragma multi_compile USE_SHAPE_LIGHT_TYPE_3 __

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            #include "Packages/com.unity.render-pipelines.universal/Shaders/2D/Include/LightingUtility.hlsl"

            struct Attributes
            {
                float4 positionOS : POSITION;
                float2 uv : TEXCOORD0;
                float4 color : COLOR;
            };

            struct Varyings
            {
                float4 positionHCS : SV_POSITION;
                float2 uv : TEXCOORD0;
                float4 color : COLOR;
                float3 worldPos : TEXCOORD1;
                float4 screenPos : TEXCOORD2;
                half2 lightingUV : TEXCOORD3;
            };

            TEXTURE2D(_MainTex);
            TEXTURE2D(_UnitDepthTex);
            SAMPLER(sampler_MainTex);
            SAMPLER(sampler_UnitDepthTex);

            #if USE_SHAPE_LIGHT_TYPE_0
            SHAPE_LIGHT(0)
            #endif
            #if USE_SHAPE_LIGHT_TYPE_1
            SHAPE_LIGHT(1)
            #endif
            #if USE_SHAPE_LIGHT_TYPE_2
            SHAPE_LIGHT(2)
            #endif
            #if USE_SHAPE_LIGHT_TYPE_3
            SHAPE_LIGHT(3)
            #endif

            float4 _Color;
            float4 _WallBasePos;
            float4 _WallDepthDir;
            float _DepthBias;
            float _DebugShowDepth;
            half _HDREmulationScale;

            Varyings vert(Attributes v)
            {
                Varyings o;
                VertexPositionInputs pos = GetVertexPositionInputs(v.positionOS.xyz);
                o.positionHCS = pos.positionCS;
                o.worldPos = pos.positionWS;
                o.uv = v.uv;
                o.color = v.color;
                o.screenPos = ComputeScreenPos(o.positionHCS);
                o.lightingUV = half2(ComputeScreenPos(o.positionHCS / o.positionHCS.w).xy);
                return o;
            }

            half4 frag(Varyings i) : SV_Target
            {
                half4 tex = SAMPLE_TEXTURE2D(_MainTex, sampler_MainTex, i.uv);
                half4 col = tex * i.color * _Color;

                if (col.a < 0.01)
                    discard;

                float2 screenUV = i.screenPos.xy / i.screenPos.w;

                // 디버그: 깊이 비교 시각화
                if (_DebugShowDepth > 0.5)
                {
                    float2 unitPos = SAMPLE_TEXTURE2D(_UnitDepthTex, sampler_UnitDepthTex, screenUV).rg;

                    // 유닛이 없는 영역은 파란색
                    if (abs(unitPos.x) < 0.001 && abs(unitPos.y) < 0.001)
                        return half4(0, 0, 1, 1);

                    // 벽의 투영 방향으로 깊이 계산
                    float unitDepth = dot(unitPos, _WallDepthDir.xy);
                    float wallDepth = dot(_WallBasePos.xy, _WallDepthDir.xy);

                    // 비교 결과 시각화:
                    // 초록 = 벽이 앞 (오클루전 O)
                    // 빨강 = 유닛이 앞 (오클루전 X)
                    if (wallDepth + _DepthBias < unitDepth)
                        return half4(0, 1, 0, 1);
                    else
                        return half4(1, 0, 0, 1);
                }

                // 2D 라이팅 적용
                half4 finalModulate = half4(1, 1, 1, 1);
                half4 finalAdditive = half4(0, 0, 0, 0);

                #if USE_SHAPE_LIGHT_TYPE_0
                half4 shapeLight0 = SAMPLE_TEXTURE2D(_ShapeLightTexture0, sampler_ShapeLightTexture0, i.lightingUV);
                finalModulate += shapeLight0 * _ShapeLightBlendFactors0.x;
                finalAdditive += shapeLight0 * _ShapeLightBlendFactors0.y;
                #endif

                #if USE_SHAPE_LIGHT_TYPE_1
                half4 shapeLight1 = SAMPLE_TEXTURE2D(_ShapeLightTexture1, sampler_ShapeLightTexture1, i.lightingUV);
                finalModulate += shapeLight1 * _ShapeLightBlendFactors1.x;
                finalAdditive += shapeLight1 * _ShapeLightBlendFactors1.y;
                #endif

                #if USE_SHAPE_LIGHT_TYPE_2
                half4 shapeLight2 = SAMPLE_TEXTURE2D(_ShapeLightTexture2, sampler_ShapeLightTexture2, i.lightingUV);
                finalModulate += shapeLight2 * _ShapeLightBlendFactors2.x;
                finalAdditive += shapeLight2 * _ShapeLightBlendFactors2.y;
                #endif

                #if USE_SHAPE_LIGHT_TYPE_3
                half4 shapeLight3 = SAMPLE_TEXTURE2D(_ShapeLightTexture3, sampler_ShapeLightTexture3, i.lightingUV);
                finalModulate += shapeLight3 * _ShapeLightBlendFactors3.x;
                finalAdditive += shapeLight3 * _ShapeLightBlendFactors3.y;
                #endif

                #if USE_SHAPE_LIGHT_TYPE_0 || USE_SHAPE_LIGHT_TYPE_1 || USE_SHAPE_LIGHT_TYPE_2 || USE_SHAPE_LIGHT_TYPE_3
                col.rgb = _HDREmulationScale * (col.rgb * finalModulate.rgb + finalAdditive.rgb);
                #endif

                return col;
            }
            ENDHLSL
        }
    }
}
