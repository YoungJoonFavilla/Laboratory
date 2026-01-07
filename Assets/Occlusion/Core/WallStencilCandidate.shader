Shader "Custom/WallStencilCandidate"
{
    Properties
    {
        _MainTex("Texture", 2D) = "white" {}
        _DepthMap("Depth Map", 2D) = "black" {}
        _Color("Color", Color) = (1,1,1,1)
        _DepthDir("Depth Direction", Vector) = (-0.636, 0.772, 0, 0)
        _DepthOffset("Depth Offset", Float) = 0
        _DepthRange("Depth Range", Float) = 3.0
    }

    SubShader
    {
        Tags
        {
            "RenderPipeline"="UniversalPipeline"
            "Queue"="Geometry-1"
            "RenderType"="Transparent"
        }

        // Pass 1: 스텐실 마킹 (색 출력 안 함)
        Pass
        {
            Name "WallStencil"
            Tags { "LightMode"="SRPDefaultUnlit" }
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
            };

            TEXTURE2D(_MainTex);
            TEXTURE2D(_DepthMap);
            SAMPLER(sampler_MainTex);
            SAMPLER(sampler_DepthMap);
            float4 _DepthDir;
            float _DepthOffset;
            float _DepthRange;
            float _UnitDepth; // 글로벌 변수 - 유닛의 균일 깊이

            Varyings vert(Attributes v)
            {
                Varyings o;
                VertexPositionInputs pos = GetVertexPositionInputs(v.positionOS.xyz);
                o.positionHCS = pos.positionCS;
                o.worldPos = pos.positionWS;
                o.uv = v.uv;
                return o;
            }

            half4 frag(Varyings i) : SV_Target
            {
                half4 tex = SAMPLE_TEXTURE2D(_MainTex, sampler_MainTex, i.uv);
                if (tex.a < 0.01)
                    discard;

                // 깊이맵 샘플링 (검정=0=앞, 흰색=1=뒤)
                float depthMapValue = SAMPLE_TEXTURE2D(_DepthMap, sampler_DepthMap, i.uv).r;

                // 벽 픽셀의 깊이 계산 (깊이맵 + 오프셋 적용)
                float wallPixelDepth = dot(i.worldPos.xy, _DepthDir.xy) + _DepthOffset + (depthMapValue * _DepthRange);

                // 벽 픽셀 깊이 < 유닛 깊이 → 벽이 앞에 있음 → 스텐실 마킹
                // 벽 픽셀 깊이 >= 유닛 깊이 → 유닛이 앞에 있음 → 마킹 안함
                if (wallPixelDepth >= _UnitDepth)
                    discard;

                return 0;
            }
            ENDHLSL
        }

        // Pass 2: 텍스처 렌더링
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

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"

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
            };

            TEXTURE2D(_MainTex);
            SAMPLER(sampler_MainTex);
            float4 _Color;

            Varyings vert(Attributes v)
            {
                Varyings o;
                VertexPositionInputs pos = GetVertexPositionInputs(v.positionOS.xyz);
                o.positionHCS = pos.positionCS;
                o.uv = v.uv;
                o.color = v.color;
                return o;
            }

            half4 frag(Varyings i) : SV_Target
            {
                half4 tex = SAMPLE_TEXTURE2D(_MainTex, sampler_MainTex, i.uv);
                half4 col = tex * i.color * _Color;

                if (col.a < 0.01)
                    discard;

                return col;
            }
            ENDHLSL
        }
    }
}
