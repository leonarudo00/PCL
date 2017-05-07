#version 150 core
in			vec4	position;			// ローカル座標系での頂点位置
in			vec4	normal;				// ローカル座標系での頂点法線
out			vec4	p;					// positionのxyz値をフラグメントシェーダに渡すためのもの
out			vec4	n;					// 正規化されたローカル座標系での頂点法線
uniform		float	scale;				// 拡縮率
uniform		vec2	location;			// 正規化デバイス座標系上での位置
uniform		vec2	size;				// ウィンドウのサイズ
uniform		mat4	projectionMatrix;	// 透視投影変換行列
uniform		mat4	transMatrixView;	// 視野変換行列
uniform		mat4	transMatrixNormal;	// 法線をワールド座標系に変換する行列

void main()
{
	//p = position;
	p = transMatrixView * position;

	//n = normal;
	n = transMatrixNormal * normal;

	gl_Position = projectionMatrix * position * vec4(2.0 * scale / size, 1.0, 1.0) + vec4(location, 0.0, 0.0);
}