#version 150 core
in			vec4	position;			// ���[�J�����W�n�ł̒��_�ʒu
in			vec4	normal;				// ���[�J�����W�n�ł̒��_�@��
out			vec4	p;					// ���_���W�n�ł̒��_�ʒu
out			vec4	n;					// ���_���W�n�ł̒��_�@��
uniform		float	scale;				// �g�k��
uniform		vec2	location;			// ���K���f�o�C�X���W�n��ł̈ʒu
uniform		vec2	size;				// �E�B���h�E�̃T�C�Y
uniform		mat4	projectionMatrix;	// �������e�ϊ��s��
uniform		mat4	transNormalMat;		// �@�������[���h���W�n�ɕϊ�����s��
uniform		mat4	transViewMat;		// ����ϊ��s��

void main()
{
	// ���_���W�n�ɕϊ��������_�ʒu���擾����
	p = transViewMat * position;

	// ���_���W�n�ɕϊ��������_�@�����擾����
	n = transNormalMat * normal;

	gl_Position = projectionMatrix * position * vec4(2.0 * scale / size, 1.0, 1.0) + vec4(location, 0.0, 0.0);
}