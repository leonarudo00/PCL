#version 150 core
uniform		vec2	size;
uniform		float	scale;
uniform		vec2	location;
uniform		mat4	projectionMatrix;	// 透視投影変換行列
in			vec4	position;			// ローカル座標系での頂点位置
in			vec4	normal;				// ローカル座標系での頂点法線
out			vec3	vc;					// 頂点色
out			vec4	n;
out			mat4	mp;
out			vec3	p;

void main()
{
	mp = projectionMatrix;
	p = position.xyz;

	n = normalize(normal);

	float z = position.z * 6.0 + 2.0;
	vc = clamp(vec3(z - 2.0, 2.0 - abs(z - 2.0), 2.0 - z), 0.0, 1.0);

	gl_Position = projectionMatrix * position * vec4(2.0 * scale / size, 1.0, 1.0) + vec4(location, 0.0, 0.0);
}