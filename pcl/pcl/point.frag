#version 150 core
out		vec4		fragment;	// 画素の色
in		vec3		vc;			// 頂点色の補間値
in		vec4		n;			// ローカル座標系での頂点法線
uniform	sampler2D	imap;

void main()
{
	// テクスチャ座標をもとめる
	vec2 st = n.xz / ( 2.0f * ( 1 + n.y ) ) + 0.5;

	// 放射照度マップのカラーを取得
	vec4 irrColor = texture( imap, st );

	fragment = vec4( irrColor );
}