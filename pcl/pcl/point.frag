#version 150 core
out vec4	fragment;	// ��f�̐F
in	vec3	vc;			// ���_�F�̕�Ԓl

void main()
{
	fragment = vec4( vc, 1.0 );
}