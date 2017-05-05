#version 150 core
in			vec4	position;			// ���[�J�����W�n�ł̒��_�ʒu
in			vec4	normal;				// ���[�J�����W�n�ł̒��_�@��
out			vec3	p;					// position��xyz�l���t���O�����g�V�F�[�_�ɓn�����߂̂���
out			vec4	n;					// ���K�����ꂽ���[�J�����W�n�ł̒��_�@��
uniform		float	scale;				// �g�k��
uniform		vec2	location;			// ���K���f�o�C�X���W�n��ł̈ʒu
uniform		vec2	size;				// �E�B���h�E�̃T�C�Y
uniform		mat4	projectionMatrix;	// �������e�ϊ��s��

void main()
{
	p = position.xyz;

	n = normalize(normal);

	float z = position.z * 6.0 + 2.0;

	gl_Position = projectionMatrix * position * vec4(2.0 * scale / size, 1.0, 1.0) + vec4(location, 0.0, 0.0);
}