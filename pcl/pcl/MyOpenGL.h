#pragma once
#define pi	3.14159265f	// �~����
#include <iostream>
#include <vector>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

namespace MyOpenGL{
	class MyOpenGL
	{
	public:
		// ���s���e�ϊ��s������߂�
		void orthogonalMatrix( float left, float right, float bottom, float top, float Near, float Far, GLfloat *matrix )
		{
			float dx = right - left;
			float dy = top - bottom;
			float dz = Far - Near;

			matrix[ 0 ] = 2.0f / dx;
			matrix[ 5 ] = 2.0f / dy;
			matrix[ 10 ] = -2.0f / dz;
			matrix[ 12 ] = -( right + left ) / dx;
			matrix[ 13 ] = -( top + bottom ) / dy;
			matrix[ 14 ] = -( Far + Near ) / dz;
			matrix[ 15 ] = 1.0f;
			matrix[ 1 ] = matrix[ 2 ] = matrix[ 3 ] = matrix[ 4 ] = matrix[ 6 ] = matrix[ 7 ] = matrix[ 8 ] = matrix[ 9 ] = matrix[ 11 ] = 0.0f;

		}

		// �������e�ϊ��s������߂�
		void perspectiveMatrix( float left, float right, float bottom, float top, float Near, float Far, GLfloat *matrix )
		{
			float dx = right - left;
			float dy = top - bottom;
			float dz = Far - Near;

			matrix[ 0 ] = 2.0f * Near / dx;
			matrix[ 5 ] = 2.0f * Near / dy;
			matrix[ 8 ] = ( right + left ) / dx;
			matrix[ 9 ] = ( top + bottom ) / dy;
			matrix[ 10 ] = -( Far + Near ) / dz;
			matrix[ 11 ] = -1.0f;
			matrix[ 14 ] = -2.0f * Far * Near / dz;
			matrix[ 1 ] = matrix[ 2 ] = matrix[ 3 ] = matrix[ 4 ] = matrix[ 6 ] = matrix[ 7 ] = matrix[ 12 ] = matrix[ 13 ] = matrix[ 15 ] = 0.0f;

		}

		// ����ϊ��s������߂�
		void lookAt( float ex, float ey, float ez, float tx, float ty, float tz, float ux, float uy, float uz, GLfloat *matrix )
		{
			float l;

			// z�� = e - t
			tx = ex - tx;
			ty = ey - ty;
			tz = ez - tz;
			l = sqrtf( tx * tx + ty * ty + tz * tz );
			matrix[ 2 ] = tx / l;
			matrix[ 6 ] = ty / l;
			matrix[ 10 ] = tz / l;

			// x�� = u x z
			tx = uy * matrix[ 10 ] - uz * matrix[ 6 ];
			ty = uz * matrix[ 2 ] - ux * matrix[ 10 ];
			tz = ux * matrix[ 6 ] - uy * matrix[ 2 ];
			l = sqrtf( tx * tx + ty * ty + tz * tz );
			matrix[ 0 ] = tx / l;
			matrix[ 4 ] = ty / l;
			matrix[ 8 ] = tz / l;

			// y�� = z�� x x��
			matrix[ 1 ] = matrix[ 6 ] * matrix[ 8 ] - matrix[ 10 ] * matrix[ 4 ];
			matrix[ 5 ] = matrix[ 10 ] * matrix[ 0 ] - matrix[ 2 ] * matrix[ 8 ];
			matrix[ 9 ] = matrix[ 2 ] * matrix[ 4 ] - matrix[ 6 ] * matrix[ 0 ];

			// ���s�ړ�
			matrix[ 12 ] = -( ex * matrix[ 0 ] + ey * matrix[ 4 ] + ez * matrix[ 8 ] );
			matrix[ 13 ] = -( ex * matrix[ 1 ] + ey * matrix[ 5 ] + ez * matrix[ 9 ] );
			matrix[ 14 ] = -( ex * matrix[ 2 ] + ey * matrix[ 6 ] + ez * matrix[ 10 ] );

			// �c��
			matrix[ 3 ] = matrix[ 7 ] = matrix[ 11 ] = 0.0f;
			matrix[ 15 ] = 1.0f;

		}

		// �s��̐ς����߂�
		void multiplyMatrix( const GLfloat *m0, const GLfloat *m1, GLfloat *matrix )
		{
			for ( int i = 0; i < 16; i++ ){
				int j = i & ~3, k = i & 3;

				matrix[ i ] = m0[ j + 0 ] * m1[ 0 + k ]
					+ m0[ j + 1 ] * m1[ 4 + k ]
					+ m0[ j + 2 ] * m1[ 8 + k ]
					+ m0[ j + 3 ] * m1[ 12 + k ];
			}
		}

		// ��p���瓧�����e�ϊ��s������߂�
		void cameraMatrix( float fovy, float aspect, float Near, float Far, GLfloat *matrix )
		{
			float f = 1.0f / tanf( fovy * 0.5f * 3.141593f / 180.0f );
			float dz = Far - Near;

			matrix[ 0 ] = f / aspect;
			matrix[ 5 ] = f;
			matrix[ 10 ] = -( Far + Near ) / dz;
			matrix[ 11 ] = -1.0f;
			matrix[ 14 ] = -2.0f * Far * Near / dz;
			matrix[ 1 ] = matrix[ 2 ] = matrix[ 3 ] = matrix[ 4 ] =
				matrix[ 6 ] = matrix[ 7 ] = matrix[ 8 ] = matrix[ 9 ] =
				matrix[ 12 ] = matrix[ 13 ] = matrix[ 15 ] = 0.0f;
		}

	};

	// �x�N�g��
	struct vec3f
	{
		float x, y, z;
	};

	// �ʃf�[�^
	struct index
	{
		GLuint position[ 3 ];	// ���_���W�ԍ�
		GLuint normal[ 3 ];		// ���_�@���ԍ�
		GLuint texture[ 3 ];	// �e�N�X�`�����W�ԍ�
		bool smooth;			// �X���[�Y�V�F�[�f�B���O�̗L��
	};

	// OBJ�t�@�C����ǂݍ���
	// name:		OBJ�t�@�C����
	// vertexNum:	�ǂݍ��񂾃f�[�^�̒��_�����i�[����ϐ�
	// pos:			���_�̈ʒu�̃f�[�^���i�[�����������̃|�C���^���i�[����ϐ�
	// norm:		���_�̖@���f�[�^���i�[�����������̃|�C���^���i�[����ϐ�
	// faceNum:		�ǂݍ��񂾃f�[�^�̖ʐ����i�[����ϐ�
	// face:		�ʂ̃f�[�^���i�[�����������̃|�C���^���i�[����ϐ�
	// normalize:	true�Ȃ�T�C�Y�𐳋K������
	// return:		�ǂݍ��݂ɐ���������true
	bool loadOBJ( const char *name, GLuint &vertexNum, GLfloat( *&pos )[ 3 ], GLfloat( *&norm )[ 3 ],
		GLuint &faceNum, GLuint( *&face )[ 3 ], bool normalize )
	{
		// OBJ�t�@�C���̓ǂݍ���
		std::ifstream file( name, std::ios::binary );

		// �t�@�C�����J���Ȃ�������߂�
		if ( !file ){
			std::cerr << "Error: Can't open OBJ file: " << name << std::endl;
			return false;
		}

		// ��s�ǂݍ��ݗp�̃o�b�t�@
		std::string line;

		// �f�[�^�̐��ƍ��W�l�̍ŏ��l�E�ő�l
		float xmin, xmax, ymin, ymax, zmin, zmax;
		xmax = ymax = zmax = -( xmin = ymin = zmin = FLT_MAX );

		// ���_�ʒu�̈ꎞ�ۑ�
		std::vector<vec3f> tpos;
		std::vector<index> tface;

		// �f�[�^��ǂݍ���
		while ( std::getline( file, line ) ){
			std::istringstream str( line );
			std::string op;
			str >> op;

			if ( op == "v" ){
				// ���_�ʒu
				vec3f v;

				// ���_�ʒu�̓X�y�[�X�ŋ�؂��Ă���
				str >> v.x >> v.y >> v.z;

				// �ʒu�̍ő�l�ƍŏ��l�����߂�
				xmin = std::min( xmin, v.x );
				xmax = std::min( xmax, v.x );
				ymin = std::min( ymin, v.y );
				ymax = std::min( ymax, v.y );
				zmin = std::min( zmin, v.z );
				zmax = std::min( zmax, v.z );

				// ���_�f�[�^��ۑ�����
				tpos.push_back( v );
			}
			else if ( op == "f" ){
				// �ʃf�[�^
				index f;
			}
		}
	}
}