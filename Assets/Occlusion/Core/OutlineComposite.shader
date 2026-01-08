Shader "Custom/OutlineComposite"
{
    Properties
    {
        _OutlineColor("Outline Color", Color) = (1, 0.92, 0.016, 1)
        _OutlineWidth("Outline Width", Float) = 3
    }

    SubShader
    {
        Tags { "RenderPipeline" = "UniversalPipeline" }

        Pass
        {
            Name "OutlineComposite"

            ZWrite Off
            ZTest Always
            Cull Off
            Blend SrcAlpha OneMinusSrcAlpha

            HLSLPROGRAM
            #pragma vertex Vert
            #pragma fragment frag

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            #include "Packages/com.unity.render-pipelines.core/Runtime/Utilities/Blit.hlsl"

            TEXTURE2D(_MaskTex);
            SAMPLER(sampler_MaskTex);

            float4 _MaskTex_TexelSize;
            float4 _OutlineColor;
            float _OutlineWidth;

            half4 frag(Varyings i) : SV_Target
            {
                float2 uv = i.texcoord;

                // 현재 픽셀이 마스크 안에 있으면 외곽선 아님
                half centerMask = SAMPLE_TEXTURE2D(_MaskTex, sampler_MaskTex, uv).r;
                if (centerMask > 0.5)
                    return half4(0, 0, 0, 0);

                // 8방향 샘플링으로 주변에 마스크가 있는지 확인
                float2 texelSize = _MaskTex_TexelSize.xy * _OutlineWidth;

                half maxMask = 0;
                maxMask = max(maxMask, SAMPLE_TEXTURE2D(_MaskTex, sampler_MaskTex, uv + float2(-texelSize.x, 0)).r);
                maxMask = max(maxMask, SAMPLE_TEXTURE2D(_MaskTex, sampler_MaskTex, uv + float2(texelSize.x, 0)).r);
                maxMask = max(maxMask, SAMPLE_TEXTURE2D(_MaskTex, sampler_MaskTex, uv + float2(0, -texelSize.y)).r);
                maxMask = max(maxMask, SAMPLE_TEXTURE2D(_MaskTex, sampler_MaskTex, uv + float2(0, texelSize.y)).r);
                maxMask = max(maxMask, SAMPLE_TEXTURE2D(_MaskTex, sampler_MaskTex, uv + float2(-texelSize.x, -texelSize.y)).r);
                maxMask = max(maxMask, SAMPLE_TEXTURE2D(_MaskTex, sampler_MaskTex, uv + float2(texelSize.x, -texelSize.y)).r);
                maxMask = max(maxMask, SAMPLE_TEXTURE2D(_MaskTex, sampler_MaskTex, uv + float2(-texelSize.x, texelSize.y)).r);
                maxMask = max(maxMask, SAMPLE_TEXTURE2D(_MaskTex, sampler_MaskTex, uv + float2(texelSize.x, texelSize.y)).r);

                // 주변에 마스크가 있으면 외곽선
                if (maxMask > 0.5)
                    return _OutlineColor;

                return half4(0, 0, 0, 0);
            }
            ENDHLSL
        }
    }
}
