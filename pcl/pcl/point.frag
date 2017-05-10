#version 150 core
#extension GL_ARB_explicit_attrib_location : enable

// THETA�摜�̃}�b�s���O���@
// 0 : ���C�u���ˏƓx�}�b�s���O
// 1 : ���}�b�s���O
// 2 : ���O�v�Z�ς݂̕��ˏƓx�}�b�s���O
#define MAPPING_MODE 0

in		vec4		p;						// ���[�J�����W�n�ł̒��_�ʒu
in		vec4		n;						// ���[�J�����W�n�ł̒��_�@��
out		vec4		fragment;				// ��f�̐F
uniform int			diffuseLod;				// �g�U���ˌ����T���v������~�b�v�}�b�v�̃��x��
uniform int			diffuseSamples;			// �g�U���ˌ��̃T���v���_�̐�
uniform int			specularLod;			// ���ʔ��ˌ����T���v������~�b�v�}�b�v�̃��x��
uniform int			specularSamples;		// ���ʔ��ˌ��̃T���v���_�̐�
uniform float		radius;					// �T���v���_�̎U�z���a
uniform	sampler2D	irrmap;					// ���ˏƓx�}�b�v
uniform sampler2D	envImage;				// ���̃e�N�X�`��

// ���e�N�X�`���̃T�C�Y
vec2 size = textureSize(envImage,0);	

// �ꖇ�̋���摜�̃A�X�y�N�g��
float aspect = ( size.x * 0.5 ) / size.y;

 // �p(=��/2)�̋t��
const float invElevationAngle = 0.63661977;

// ���e�N�X�`���̑O���J�������̃e�N�X�`����ԏ�̔��a�ƒ��S
// ����Ԃ̓���̓_���w�� radius_f �� radius_b ��x�����]�B�Ȃ̂� radius_f.x = -0.25
vec2 radius_f = vec2( -0.25, 0.5 * aspect );	
vec2 center_f = vec2( 0.25, radius_f.t );

// ���e�N�X�`���̌���J�������̃e�N�X�`����ԏ�̔��a�ƒ��S
vec2 radius_b = vec2( 0.25, radius_f.t );	
vec2 center_b = vec2( 0.75, center_f.t );

// DualFisheye�摜�̃T���v�����O
// lod : �~�b�v�}�b�v�̃��x��
vec4 sample( vec3 vector, int lod )
{
	// vector�̑O���J�������ʕ����ɑ΂���V���p
	// �V���p = acos( vector.z / ||vector|| )
	// [�O��,���]�J�����V�������Ȃ�vector.z = [1.0,-1.0]�Aacos(vector.z) = [0,��]
	float zenithAngle = acos( vector.z );

	// �p�ƓV���p�̔䗦��max/��
	// �f���Ƀ�max/�Ƃł��Ƃ߂�ƃ[��������������\��������̂ł�≓�܂킵�B
	float angleRate = 1.0 - zenithAngle * invElevationAngle;

	// ���ۂ̉�p230������180���̗̈�ɐ��K�����ꂽ
	// vector�� yx ��ł̕��ʃx�N�g��
	vec2 orientation = normalize(vector.yx) * sqrt(180.0f / 230.0f);

	// ���ƕ\�̃e�N�X�`�����W�����߂�
	vec2 t_f = ( 1.0 - angleRate ) * radius_f * orientation + center_f;
	vec2 t_b = ( 1.0 + angleRate ) * radius_b * orientation + center_b;

	// ���ƕ\�̊��}�b�v���T���v�����O����
	vec4 color_f = textureLod(envImage, t_f, lod);
	vec4 color_b = textureLod(envImage, t_b, lod);

	// �O��̃e�N�X�`���̍�����
	// �G���~�[�g��Ԃ炵�����ǂ悭�킩���
	float blend = smoothstep(-0.02, 0.02, angleRate );

	// �T���v�����O�����F���u�����h����
	return mix(color_b, color_f, blend);
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
// ����̗������悭�킩���
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

#if MAPPING_MODE == 0
	//
	// ���C�u���ˏƓx�}�b�s���O
	//

	// �T���v���_��@�������ɉ�]����ϊ��s��
	// �悭�킩���
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

		// �T���v���_�����̐F��ݐς���
		idiff += sample(l, diffuseLod);
	}
	
	// ���ς������ĕ��ˏƓx�����߂�
	idiff /= float(diffuseSamples);

	// �t���l����
	vec4 fresnel = vec4( vec3( 0.1 ), 1.0 );
	
	// �����x�N�g��
	vec3 v = normalize( p.xyz );

	// ���ʔ��˂̐��K���W��
	float e = 1.0 / (fresnel.a * 128.0 + 1.0);

	// ���ʔ��ˌ����x
	vec4 ispec = vec4(0.0);

	// �����˕���
	vec3 r = reflect( v, n.xyz );

	// �����˕����̐F
	vec4 spec = sample(r, 0);

	// �����ˑ��̌X�̃T���v���_�ɂ���
	for (int i = 0; i < specularSamples; ++i)
	{
		// �T���v���_�̐���
		vec4 s = sampler(seed, e);

		// �T���v���_��@�����ɉ�]�������̂�@���x�N�g���ɗp���Đ����˕��������߂�
		// �e�N�X�`���̉�]�s��͏ȗ�
		vec3 r = reflect(v, m * s.xyz);

		// �T���v���_�����̐F��ݐς���
		ispec += sample(r, specularLod);
	}

	// ���ς������ċ��ʔ��ˌ����x�����߂�
	ispec /= float(specularSamples);

	// ��f�̉A�e�����߂�
	fresnel.a = 0.0;
	vec4 color = mix( idiff, spec, fresnel );
	fragment = vec4( color.zyx, color.w );

#elif MAPPING_MODE == 1
	//
	// ���}�b�s���O
	//

	vec4 color = sample( n.xyz, diffuseLod );

	fragment = vec4( color.zyx, color.w );

#elif MAPPING_MODE == 2
	//
	// ���O�v�Z�ς݂̕��ˏƓx�}�b�s���O
	//

	// �e�N�X�`�����W�����Ƃ߂�
	vec2 st = n.xz / ( 2.0f * ( 1 + n.y ) ) + 0.5;

	// ���ˏƓx�}�b�v�̃J���[���擾
	vec4 irrColor = texture( irrmap, st );

#endif
}