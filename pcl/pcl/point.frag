#version 150 core
out		vec4		fragment;	// ��f�̐F
in		vec3		vc;			// ���_�F�̕�Ԓl
in		vec4		n;			// ���[�J�����W�n�ł̒��_�@��
uniform	sampler2D	imap;		// ���ˏƓx�}�b�v
uniform sampler2D	image;		// ���̃e�N�X�`��

vec2 size = textureSize(image,0);	// ���e�N�X�`���̃T�C�Y
// ���e�N�X�`���̌���J�������̃e�N�X�`����ԏ�̔��a�ƒ��S
vec2 radius_b = vec2( -0.25, 0.25 * size.x / size.y );	
vec2 center_b = vec2( 0.25, radius_b.t );

// ���e�N�X�`���̑O���J�������̃e�N�X�`����ԏ�̔��a�ƒ��S
vec2 radius_f = vec2( 0.25, radius_b.t );	
vec2 center_f = vec2( 0.75, center_b.t );

// ���}�b�v�̃T���v�����O
vec4 sample( vec3 vector, int lod)
{
	//
	// RICOH THETA S �̃��C�u�X�g���[�~���O�摜�̏ꍇ
	//

	// ���̕����x�N�g���̑��ΓI�ȋp
	float angle = 1.0 - acos(vector.z) * 0.63661977;

	// �O��̃e�N�X�`���̍�����
	float blend = smoothstep(-0.02, 0.02, angle);

	// ���̕����x�N�g���� yx ��ł̕����x�N�g��
	vec2 orientation = normalize(vector.yx) * 0.885;

	// ���ƕ\�̃e�N�X�`�����W�����߂�
	//vec2 t_b = (1.0 - angle) * orientation * radius_b + center_b;
	//vec2 t_f = (1.0 + angle) * orientation * radius_f + center_f;
	vec2 t_b = (1.0 - angle) * orientation * radius_b + center_b;
	vec2 t_f = (1.0 + angle) * orientation * radius_f + center_f;

	// ���ƕ\�̊��}�b�v���T���v�����O����
	vec4 color_b = textureLod(image, t_b, lod);
	vec4 color_f = textureLod(image, t_f, lod);

	// �T���v�����O�����F���u�����h����
	return mix(color_f, color_b, blend);
}

void main()
{
	// �e�N�X�`�����W�����Ƃ߂�
	vec2 st = n.xz / ( 2.0f * ( 1 + n.y ) ) + 0.5;

	// ���ˏƓx�}�b�v�̃J���[���擾
	vec4 irrColor = texture( imap, st );

	vec4 color = sample(n.xyz, 0);

	fragment = vec4( color.zyx,1.0 );
}