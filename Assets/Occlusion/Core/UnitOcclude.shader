Shader "Custom/UnitOcclude"
{
    Properties
    {
        _MainTex("Texture", 2D) = "white" {}
        _Color("Color", Color) = (1,1,1,1)
        _OccludedAlpha("Occluded Alpha", Range(0,1)) = 0.3
    }

    SubShader
    {
        Tags
        {
            "RenderPipeline"="UniversalPipeline"
            "Queue"="Transparent"
            "RenderType"="Transparent"
        }

        // Pass 0: 유닛 위치 텍스처에 XY 좌표 쓰기 (RenderFeature에서 호출)
        Pass
        {
            Name "UnitDepthWrite"
            Tags { "LightMode"="UnitDepthPass" }

            Blend Off
            ZWrite Off
            Cull Off

            HLSLPROGRAM
            #pragma vertex vertDepth
            #pragma fragment fragDepth

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"

            struct AttributesDepth
            {
                float4 positionOS : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct VaryingsDepth
            {
                float4 positionHCS : SV_POSITION;
                float2 uv : TEXCOORD0;
            };

            TEXTURE2D(_MainTex);
            SAMPLER(sampler_MainTex);

            VaryingsDepth vertDepth(AttributesDepth v)
            {
                VaryingsDepth o;
                VertexPositionInputs pos = GetVertexPositionInputs(v.positionOS.xyz);
                o.positionHCS = pos.positionCS;
                o.uv = v.uv;
                return o;
            }

            half4 fragDepth(VaryingsDepth i) : SV_Target
            {
                half4 tex = SAMPLE_TEXTURE2D(_MainTex, sampler_MainTex, i.uv);
                if (tex.a < 0.01)
                    discard;

                // 오브젝트의 Transform 위치 (발 위치) XY 좌표 출력
                float2 objectPos = float2(UNITY_MATRIX_M._m03, UNITY_MATRIX_M._m13);

                return half4(objectPos.x, objectPos.y, 0, 1);
            }
            ENDHLSL
        }

        // Pass 1: 정상 영역 렌더링
        Pass
        {
            Name "UnitRender"
            Tags { "LightMode"="Universal2D" }

            Blend SrcAlpha OneMinusSrcAlpha
            ZWrite Off
            Cull Off

            // 스텐실≠1인 영역에서만 렌더링 (벽에 가려지지 않은 부분)
            Stencil
            {
                Ref 1
                Comp NotEqual
                Pass Keep
            }

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
            float _OccludedAlpha;

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

        // Pass 2: 가려진 영역 - 반투명 렌더링
        Pass
        {
            Name "UnitOccluded"
            Tags { "LightMode"="UnitOccludedPass" }

            Blend SrcAlpha OneMinusSrcAlpha
            ZWrite Off
            Cull Off

            // 스텐실=1인 영역 (벽에 가려진 부분)
            Stencil
            {
                Ref 1
                Comp Equal
                Pass Keep
            }

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
            float _OccludedAlpha;

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

                col.a *= _OccludedAlpha;
                return col;
            }
            ENDHLSL
        }
    }
}
