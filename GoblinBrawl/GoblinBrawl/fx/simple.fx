cbuffer cbPerObject {
	float4x4 gWorldViewProj;
};

struct VertexIn {
	float3 PosL  : POSITION;
};

struct VertexOut {
	float4 PosH  : SV_POSITION;
	float4 Color : COLOR;
};

VertexOut VS( VertexIn vin ) {
	VertexOut vout;

	// Transform to homogeneous clip space.
	vout.PosH = mul( float4(vin.PosL, 1.0f), gWorldViewProj );

	//float3 nPos = normalize( vin.PosL );
	//vout.Color = float4(nPos, 1.0f);
	vout.Color = float4(1.f, 0.f, 0.f, 1.f);

	return vout;
}

float4 PS( VertexOut pin ) : SV_Target
{
	return pin.Color;
}

technique11 SimpleTech {
	pass P0 {
		SetVertexShader( CompileShader(vs_5_0, VS()) );
		SetGeometryShader( NULL );
		SetPixelShader( CompileShader(ps_5_0, PS()) );
	}
}