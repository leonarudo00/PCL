#version 150 core
out vec4	fragment;	// 画素の色
in	vec3	vc;			// 頂点色の補間値

void main()
{
	fragment = vec4( vc, 1.0 );
}