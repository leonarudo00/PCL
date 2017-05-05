#version 150 core
in			vec4	position;			// ローカル座標系での頂点位置
in			vec4	normal;				// ローカル座標系での頂点法線
out			vec3	p;					// positionのxyz値をフラグメントシェーダに渡すためのもの
out			vec4	n;					// 正規化されたローカル座標系での頂点法線
uniform		float	scale;				// 拡縮率
uniform		vec2	location;			// 正規化デバイス座標系上での位置
uniform		vec2	size;				// ウィンドウのサイズ
uniform		mat4	projectionMatrix;	// 透視投影変換行列

void main()
{
	p = position.xyz;

	n = normalize(normal);

	float z = position.z * 6.0 + 2.0;

	gl_Position = projectionMatrix * position * vec4(2.0 * scale / size, 1.0, 1.0) + vec4(location, 0.0, 0.0);
}