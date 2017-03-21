#version 150 core
out		vec4		fragment;	// ��f�̐F
in		vec3		vc;			// ���_�F�̕�Ԓl
in		vec4		n;			// ���[�J�����W�n�ł̒��_�@��
uniform	sampler2D	imap;

void main()
{
	// �e�N�X�`�����W�����Ƃ߂�
	vec2 st = n.xz / ( 2.0f * ( 1 + n.y ) ) + 0.5;

	// ���ˏƓx�}�b�v�̃J���[���擾
	vec4 irrColor = texture( imap, st );

	fragment = vec4( irrColor );
}