#version 150 core
#extension GL_ARB_explicit_attrib_location : enable

out		vec4		fragment;		// ��f�̐F
in		vec3		vc;				// ���_�F�̕�Ԓl
in		vec4		n;				// ���[�J�����W�n�ł̒��_�@��
in		mat4		mp;
in		vec3		p;
uniform	sampler2D	imap;			// ���ˏƓx�}�b�v
uniform sampler2D	image;			// ���̃e�N�X�`��
uniform int			diffuseSamples;	// �@�������̃T���v���_�̐�
uniform int			diffuseLod;		// �@�������̃~�b�v�}�b�v�̃��x��
uniform float		radius;			// �T���v���_�̎U�z���a

// ���e�N�X�`���̃T�C�Y
vec2 size = textureSize(image,0);	

// ���e�N�X�`���̌���J�������̃e�N�X�`����ԏ�̔��a�ƒ��S
vec2 radius_b = vec2( -0.25, 0.25 * size.x / size.y );	
vec2 center_b = vec2( 0.25, radius_b.t );

// ���e�N�X�`���̑O���J�������̃e�N�X�`����ԏ�̔��a�ƒ��S
vec2 radius_f = vec2( 0.25, radius_b.t );	
vec2 center_f = vec2( 0.75, center_b.t );

// ���}�b�v�̃T���v�����O
vec4 sample( vec3 vector, int lod )
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
	vec2 t_b = (1.0 - angle) * orientation * radius_b + center_b;
	vec2 t_f = (1.0 + angle) * orientation * radius_f + center_f;

	// ���ƕ\�̊��}�b�v���T���v�����O����
	vec4 color_b = textureLod(image, t_b, lod);
	vec4 color_f = textureLod(image, t_f, lod);

	// �T���v�����O�����F���u�����h����
	return mix(color_f, color_b, blend);
}

// �m�C�Y����
uint rand(in vec2 co)
{
	return uint(fract(sin(dot(co, vec2(12.9898, 78.233))) * 43758.5453) * 4294967293.0) + 1u;
}

// ��������
float xorshift(inout uint y)
{
	// shift ���� xor ����
	y = y ^ (y << 13);
	y = y ^ (y >> 17);
	y = y ^ (y << 5);

	// [0, 1] �ɐ��K�����ĕԂ�
	return float(y) * 2.3283064e-10;
}

// �T���v���_�̐���
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
	// �e�N�X�`�����W�����Ƃ߂�
	vec2 st = n.xz / ( 2.0f * ( 1 + n.y ) ) + 0.5;

	// ���ˏƓx�}�b�v�̃J���[���擾
	vec4 irrColor = texture( imap, st );

	// �T���v���_��@�������ɉ�]����ϊ��s��
	vec3 zn = vec3(-n.y, n.x, 0.0);
	float len = length(zn);
	vec3 t = mix(vec3(1.0, 0.0, 0.0), zn / len, step(0.001, len));
	vec3 b = cross(n.xyz, t);
	mat3 m = mat3(t, b, n);

	// �����̃^�l
	uint seed = rand(gl_FragCoord.xy);
	//uint seed = 2463534242u;

	// ���ˏƓx
	vec4 idiff = vec4(0.0);

	// �@�����̌X�̃T���v���_�ɂ���
	for (int i = 0; i < diffuseSamples; ++i)
	{
		// �T���v���_�𐶐�����
		vec4 d = sampler(seed, 0.5);

		// �T���v���_��@�����ɉ�]����
		vec3 l = m * d.xyz;

		// �T���v���_�̈ʒu�� p ����̑��Έʒu�ɕ��s�ړ������セ�̓_�̃N���b�s���O���W�n��̈ʒu q �����߂�
		vec4 q = mp * vec4(p + l * d.w, 1.0);

		// �e�N�X�`�����W�ɕϊ�����
		q = q * 0.5 / q.w + 0.5;

		// �T���v���_�����̐F��ݐς���
		idiff += sample(l, diffuseLod);
	}

	vec4 color = sample(n.xyz, 5);

	fragment = vec4( idiff / float(diffuseSamples) );
}