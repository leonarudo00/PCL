#version 150 core
#extension GL_ARB_explicit_attrib_location : enable

// THETA画像のマッピング方法
// 0 : ライブ放射照度マッピング
// 1 : 環境マッピング
// 2 : 事前計算済みの放射照度マッピング
#define MAPPING_MODE 0

in		vec4		p;						// ローカル座標系での頂点位置
in		vec4		n;						// ローカル座標系での頂点法線
out		vec4		fragment;				// 画素の色
uniform int			diffuseLod;				// 拡散反射光をサンプルするミップマップのレベル
uniform int			diffuseSamples;			// 拡散反射光のサンプル点の数
uniform int			specularLod;			// 鏡面反射光をサンプルするミップマップのレベル
uniform int			specularSamples;		// 鏡面反射光のサンプル点の数
uniform float		radius;					// サンプル点の散布半径
uniform	sampler2D	irrmap;					// 放射照度マップ
uniform sampler2D	envImage;				// 環境のテクスチャ

// 環境テクスチャのサイズ
vec2 size = textureSize(envImage,0);	

// 一枚の魚眼画像のアスペクト比
float aspect = ( size.x * 0.5 ) / size.y;

 // 仰角(=π/2)の逆数
const float invElevationAngle = 0.63661977;

// 環境テクスチャの前方カメラ像のテクスチャ空間上の半径と中心
// 実空間の同一の点を指す radius_f と radius_b はx軸反転。なので radius_f.x = -0.25
vec2 radius_f = vec2( -0.25, 0.5 * aspect );	
vec2 center_f = vec2( 0.25, radius_f.t );

// 環境テクスチャの後方カメラ像のテクスチャ空間上の半径と中心
vec2 radius_b = vec2( 0.25, radius_f.t );	
vec2 center_b = vec2( 0.75, center_f.t );

// DualFisheye画像のサンプリング
// lod : ミップマップのレベル
vec4 sample( vec3 vector, int lod )
{
	// vectorの前方カメラ正面方向に対する天頂角
	// 天頂角 = acos( vector.z / ||vector|| )
	// [前方,後方]カメラ天頂方向ならvector.z = [1.0,-1.0]、acos(vector.z) = [0,π]
	float zenithAngle = acos( vector.z );

	// 仰角と天頂角の比率θmax/θ
	// 素直にθmax/θでもとめるとゼロ割が発生する可能性があるのでやや遠まわし。
	float angleRate = 1.0 - zenithAngle * invElevationAngle;

	// 実際の画角230°から180°の領域に正規化された
	// vectorの yx 上での方位ベクトル
	vec2 orientation = normalize(vector.yx) * sqrt(180.0f / 230.0f);

	// 裏と表のテクスチャ座標を求める
	vec2 t_f = ( 1.0 - angleRate ) * radius_f * orientation + center_f;
	vec2 t_b = ( 1.0 + angleRate ) * radius_b * orientation + center_b;

	// 裏と表の環境マップをサンプリングする
	vec4 color_f = textureLod(envImage, t_f, lod);
	vec4 color_b = textureLod(envImage, t_b, lod);

	// 前後のテクスチャの混合比
	// エルミート補間らしいけどよくわからん
	float blend = smoothstep(-0.02, 0.02, angleRate );

	// サンプリングした色をブレンドする
	return mix(color_b, color_f, blend);
}

// ノイズ発生
uint rand(in vec2 co)
{
	return uint(fract(sin(dot(co, vec2(12.9898, 78.233))) * 43758.5453) * 4294967293.0) + 1u;
}

// 乱数発生
float xorshift(inout uint y)
{
	// shift して xor する
	y = y ^ (y << 13);
	y = y ^ (y >> 17);
	y = y ^ (y << 5);

	// [0, 1] に正規化して返す
	return float(y) * 2.3283064e-10;
}

// サンプル点の生成
// これの理屈がよくわからん
vec4 sampler(inout uint seed, in float e)
{
	float z = pow(xorshift(seed), e);
	float d = sqrt(1.0 - z * z);
	float t = 6.2831853 * xorshift(seed);
	vec3 s = normalize(vec3(vec2(cos(t), sin(t)) * d, z));
	return vec4(s, radius * xorshift(seed));
	//return vec4(s, radius * pow(xorshift(seed), 0.33333333));
}

void main()
{

#if MAPPING_MODE == 0
	//
	// ライブ放射照度マッピング
	//

	// サンプル点を法線方向に回転する変換行列
	// よくわからん
	vec3 zn = vec3(-n.y, n.x, 0.0);
	float len = length(zn);
	vec3 t = mix(vec3(1.0, 0.0, 0.0), zn / len, step(0.001, len));
	vec3 b = cross(n.xyz, t);
	mat3 m = mat3(t, b, n);

	// 乱数のタネ
	uint seed = rand(gl_FragCoord.xy);
	//uint seed = 2463534242u;

	// 放射照度
	vec4 idiff = vec4(0.0);

	// 法線側の個々のサンプル点について
	for (int i = 0; i < diffuseSamples; ++i)
	{
		// サンプル点を生成する
		vec4 d = sampler(seed, 0.5);

		// サンプル点を法線側に回転する
		vec3 l = m * d.xyz;

		// サンプル点方向の色を累積する
		idiff += sample(l, diffuseLod);
	}
	
	// 平均をだして放射照度を決める
	idiff /= float(diffuseSamples);

	// フレネル項
	vec4 fresnel = vec4( vec3( 0.1 ), 1.0 );
	
	// 視線ベクトル
	vec3 v = normalize( p.xyz );

	// 鏡面反射の正規化係数
	float e = 1.0 / (fresnel.a * 128.0 + 1.0);

	// 鏡面反射光強度
	vec4 ispec = vec4(0.0);

	// 正反射方向
	vec3 r = reflect( v, n.xyz );

	// 正反射方向の色
	vec4 spec = sample(r, 0);

	// 正反射側の個々のサンプル点について
	for (int i = 0; i < specularSamples; ++i)
	{
		// サンプル点の生成
		vec4 s = sampler(seed, e);

		// サンプル点を法線側に回転したものを法線ベクトルに用いて正反射方向を求める
		// テクスチャの回転行列は省略
		vec3 r = reflect(v, m * s.xyz);

		// サンプル点方向の色を累積する
		ispec += sample(r, specularLod);
	}

	// 平均をだして鏡面反射光強度をきめる
	ispec /= float(specularSamples);

	// 画素の陰影を求める
	fresnel.a = 0.0;
	vec4 color = mix( idiff, spec, fresnel );
	fragment = vec4( color.zyx, color.w );

#elif MAPPING_MODE == 1
	//
	// 環境マッピング
	//

	vec4 color = sample( n.xyz, diffuseLod );

	fragment = vec4( color.zyx, color.w );

#elif MAPPING_MODE == 2
	//
	// 事前計算済みの放射照度マッピング
	//

	// テクスチャ座標をもとめる
	vec2 st = n.xz / ( 2.0f * ( 1 + n.y ) ) + 0.5;

	// 放射照度マップのカラーを取得
	vec4 irrColor = texture( irrmap, st );

#endif
}