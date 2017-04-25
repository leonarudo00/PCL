#version 150 core
out		vec4		fragment;	// 画素の色
in		vec3		vc;			// 頂点色の補間値
in		vec4		n;			// ローカル座標系での頂点法線
uniform	sampler2D	imap;		// 放射照度マップ
uniform sampler2D	image;		// 環境のテクスチャ

vec2 size = textureSize(image,0);	// 環境テクスチャのサイズ
// 環境テクスチャの後方カメラ像のテクスチャ空間上の半径と中心
vec2 radius_b = vec2( -0.25, 0.25 * size.x / size.y );	
vec2 center_b = vec2( 0.25, radius_b.t );

// 環境テクスチャの前方カメラ像のテクスチャ空間上の半径と中心
vec2 radius_f = vec2( 0.25, radius_b.t );	
vec2 center_f = vec2( 0.75, center_b.t );

// 環境マップのサンプリング
vec4 sample( vec3 vector, int lod)
{
	//
	// RICOH THETA S のライブストリーミング画像の場合
	//

	// この方向ベクトルの相対的な仰角
	float angle = 1.0 - acos(vector.z) * 0.63661977;

	// 前後のテクスチャの混合比
	float blend = smoothstep(-0.02, 0.02, angle);

	// この方向ベクトルの yx 上での方向ベクトル
	vec2 orientation = normalize(vector.yx) * 0.885;

	// 裏と表のテクスチャ座標を求める
	//vec2 t_b = (1.0 - angle) * orientation * radius_b + center_b;
	//vec2 t_f = (1.0 + angle) * orientation * radius_f + center_f;
	vec2 t_b = (1.0 - angle) * orientation * radius_b + center_b;
	vec2 t_f = (1.0 + angle) * orientation * radius_f + center_f;

	// 裏と表の環境マップをサンプリングする
	vec4 color_b = textureLod(image, t_b, lod);
	vec4 color_f = textureLod(image, t_f, lod);

	// サンプリングした色をブレンドする
	return mix(color_f, color_b, blend);
}

void main()
{
	// テクスチャ座標をもとめる
	vec2 st = n.xz / ( 2.0f * ( 1 + n.y ) ) + 0.5;

	// 放射照度マップのカラーを取得
	vec4 irrColor = texture( imap, st );

	vec4 color = sample(n.xyz, 0);

	fragment = vec4( color.zyx,1.0 );
}