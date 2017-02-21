#pragma once
#define pi	3.14159265f	// �~����
#include <iostream>
#include <vector>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

namespace MyOpenGL{
	// �V�F�[�_�I�u�W�F�N�g�̃R���p�C�����ʂ�\������
	// shader: �V�F�[�_�I�u�W�F�N�g��
	// str: �R���p�C���G���[�����������ꏊ������������
	GLboolean printShaderInfoLog( GLuint shader, const char *str )
	{
		// �R���p�C�����ʂ��擾����
		GLint status;
		glGetShaderiv( shader, GL_COMPILE_STATUS, &status );
		if ( status == GL_FALSE ) std::cerr << "Compile Error in " << str << std::endl;

		// �V�F�[�_�̃R���p�C�����̃��O�̒������擾����
		GLsizei bufSize;
		glGetShaderiv( shader, GL_INFO_LOG_LENGTH, &bufSize );
		if ( bufSize > 1 ){
			// �V�F�[�_�̃R���p�C�����̃��O�̓��e���擾����
			std::vector<GLchar> infoLog( bufSize );
			GLsizei length;
			glGetShaderInfoLog( shader, bufSize, &length, &infoLog[ 0 ] );
			std::cerr << &infoLog[ 0 ] << std::endl;
		}

		return static_cast<GLboolean>( status );
	}

	// �v���O�����I�u�W�F�N�g�̃����N���ʂ�\������
	// program: �v���O�����I�u�W�F�N�g��
	GLboolean printProgramInfoLog( GLuint program )
	{
		// �����N���ʂ��擾����
		GLint status;
		glGetProgramiv( program, GL_LINK_STATUS, &status );
		if ( status == GL_FALSE ) std::cerr << "Link Error." << std::endl;

		// �V�F�[�_�̃����N���̃��O�̒������擾����
		GLsizei bufSize;
		glGetProgramiv( program, GL_INFO_LOG_LENGTH, &bufSize );

		if ( bufSize > 1 ){
			// �V�F�[�_�̃����N���̃��O�̓��e���擾����
			std::vector<GLchar> infoLog( bufSize );
			GLsizei length;
			glGetProgramInfoLog( program, bufSize, &length, &infoLog[ 0 ] );
			std::cerr << &infoLog[ 0 ] << std::endl;
		}

		return static_cast<GLboolean>( status );
	}

	// �v���O�����I�u�W�F�N�g���쐬����
	// vsrc: �o�[�e�b�N�X�V�F�[�_�̃\�[�X�v���O�����̕�����
	// fsrc: �t���O�����g�V�F�[�_�̃\�[�X�v���O�����̕�����
	GLuint createProgram( const char *vsrc, const char *fsrc )
	{
		// ��̃v���O�����I�u�W�F�N�g���쐬����
		const GLuint program( glCreateProgram() );

		if ( vsrc != NULL ){
			// �o�[�e�b�N�X�V�F�[�_�̃V�F�[�_�I�u�W�F�N�g���쐬����
			const GLuint vobj( glCreateShader( GL_VERTEX_SHADER ) );
			glShaderSource( vobj, 1, &vsrc, NULL );
			glCompileShader( vobj );

			// �o�[�e�b�N�X�V�F�[�_�̃V�F�[�_�I�u�W�F�N�g���v���O�����I�u�W�F�N�g�ɑg�ݍ���
			if ( printShaderInfoLog( vobj, "vertex shader" ) ) glAttachShader( program, vobj );
			glDeleteShader( vobj );
		}

		if ( fsrc != NULL ){
			// �t���O�����g�V�F�[�_�̃V�F�[�_�I�u�W�F�N�g���쐬����
			const GLuint fobj( glCreateShader( GL_FRAGMENT_SHADER ) );
			glShaderSource( fobj, 1, &fsrc, NULL );
			glCompileShader( fobj );

			// �t���O�����g�V�F�[�_�̃V�F�[�_�I�u�W�F�N�g���v���O�����I�u�W�F�N�g�ɑg�ݍ���
			if ( printShaderInfoLog( fobj, "fragment shader" ) ) glAttachShader( program, fobj );
			glDeleteShader( fobj );
		}

		// �v���O�����I�u�W�F�N�g�������N����
		glBindAttribLocation( program, 0, "position" );
		glBindFragDataLocation( program, 0, "fragment" );
		glLinkProgram( program );

		// �쐬�����v���O�����I�u�W�F�N�g��Ԃ�
		if ( printProgramInfoLog( program ) ) return program;

		// �v���O�����I�u�W�F�N�g���쐬�ł��Ȃ���� 0 ��Ԃ�
		glDeleteProgram( program );
		return 0;
	}

	// �V�F�[�_�̃\�[�X�t�@�C����ǂݍ��񂾃�������Ԃ�
	// name: �V�F�[�_�̃\�[�X�t�@�C����
	// buffer: �ǂݍ��񂾃\�[�X�t�@�C���̃e�L�X�g
	bool readShaderSource( const char *name, std::vector<GLchar> &buffer )
	{
		// �t�@�C������NULL������
		if ( name == NULL )return false;

		// �\�[�X�t�@�C�����J��
		std::ifstream file( name, std::ios::binary );
		if ( file.fail() ){
			// �J���Ȃ�����
			std::cerr << "Error: Can't open source file: " << name << std::endl;
			return false;
		}

		// �t�@�C���̖����Ɉړ������݈ʒu�i���t�@�C���T�C�Y�j�𓾂�
		file.seekg( 0L, std::ios::end );
		GLsizei length = static_cast< GLsizei >( file.tellg() );

		// �t�@�C���T�C�Y�̃��������m��
		buffer.resize( length + 1 );

		// �t�@�C����擪����ǂݍ���
		file.seekg( 0L, std::ios::beg );
		file.read( buffer.data(), length );
		buffer[ length ] = '\0';

		if ( file.fail() ){
			// ���܂��ǂݍ��߂Ȃ�����
			std::cerr << "Error: Could not read source file: " << name << std::endl;
			file.close();
			return false;
		}

		// �ǂݍ��ݐ���
		file.close();
		return true;
	}

	// �V�F�[�_�̃\�[�X�t�@�C����ǂݍ���Ńv���O�����I�u�W�F�N�g���쐬����
	// vert: �o�[�e�b�N�X�V�F�[�_�̃\�[�X�t�@�C����
	// frag: �t���O�����g�V�F�[�_�̃\�[�X�t�@�C����
	GLuint loadProgram( const char *vert, const char *frag )
	{
		// �V�F�[�_�̃\�[�X�t�@�C����ǂݍ���
		std::vector<GLchar> vsrc;
		const bool vstat( readShaderSource( vert, vsrc ) );
		std::vector<GLchar> fsrc;
		const bool fstat( readShaderSource( frag, fsrc ) );
		// �v���O�����I�u�W�F�N�g���쐬����
		return vstat && fstat ? createProgram( vsrc.data(), fsrc.data() ) : 0;
	}

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

	// �v�f�̓���
	// a: 3�v�f�̔z��
	// b: 3�v�f�̔z��
	inline GLfloat dot3( const GLfloat *a, const GLfloat *b )
	{
		return a[ 0 ] * b[ 0 ] + a[ 1 ] * b[ 1 ] + a[ 2 ] * b[ 2 ];
	}

	// �v�f�̊O��
	// a: 3�v�f�̔z��
	// b: 3�v�f�̔z��
	// c: ���ʂ��i�[����3�v�f�̔z��
	inline void cross( GLfloat *c, const GLfloat *a, const GLfloat *b )
	{
		c[ 0 ] = a[ 1 ] * b[ 2 ] - a[ 2 ] * b[ 1 ];
		c[ 1 ] = a[ 2 ] * b[ 0 ] - a[ 0 ] * b[ 2 ];
		c[ 2 ] = a[ 0 ] * b[ 1 ] - a[ 1 ] * b[ 0 ];
	}

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
			// �������ǂݍ���
			std::istringstream str( line );
			// �������W�J
			std::string op;
			str >> op;

			// ���_�f�[�^�̓ǂݍ���
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
			// �ʃf�[�^�̓ǂݍ���
			else if ( op == "f" ){
				// �ʃf�[�^
				index f;

				// ���_���W�ԍ������o��
				for ( int i = 0; i < 3; ++i ){
					// �P�s���X�y�[�X�ŋ�؂��ČX�̗v�f�̍ŏ��̐��l�����o��
					std::string s;
					str >> s;
					f.position[ i ] = atoi( s.c_str() );
				}

				// �ʃf�[�^��ۑ�����
				tface.push_back( f );
			}
		}

		// �t�@�C���̓ǂݍ��݃`�F�b�N
		if ( file.bad() ){
			// ���܂��ǂݍ��߂Ȃ�����
			std::cerr << "Warning: Can't read OBJ file; " << name << std::endl;
		}

		// �t�@�C�������
		file.close();

		// �������̊m��
		pos = norm = NULL;
		face = NULL;
		vertexNum = static_cast< GLuint >( tpos.size() );
		faceNum = static_cast< GLuint >( tface.size() );
		try{
			pos = new GLfloat[ vertexNum ][ 3 ];
			norm = new GLfloat[ vertexNum ][ 3 ];
			face = new GLuint[ faceNum ][ 3 ];
		}
		catch ( std::bad_alloc e ){
			delete[] pos;
			delete[] norm;
			delete[] face;

			pos = norm = NULL;
			face = NULL;

			return false;
		}

		// �ʒu�Ƒ傫���̐��K���̂��߂̌W��
		GLfloat scale, centerX, centerY, centerZ;
		if ( normalize ){
			const float sx( xmax - xmin );
			const float sy( ymax - ymin );
			const float sz( zmax - zmin );

			scale = sx;
			if ( sy > scale ) scale = sy;
			if ( sz > scale ) scale = sz;
			scale = ( scale != 0.0f ) ? 2.0f / scale : 1.0f;

			centerX = ( xmax + xmin ) * 0.5;
			centerY = ( ymax + ymin ) * 0.5;
			centerZ = ( zmax + zmin ) * 0.5;
		}
		else{
			scale = 1.0f;
			centerX = centerY = centerZ = 0.0f;
		}

		// �}�`�̑傫���ƈ�̐��K���ƃf�[�^�̃R�s�[
		for ( std::vector<vec3f>::const_iterator it = tpos.begin(); it != tpos.end(); ++it ){
			const size_t v = it - tpos.begin();

			pos[ v ][ 0 ] = ( it->x - centerX ) * scale;
			pos[ v ][ 1 ] = ( it->y - centerY ) * scale;
			pos[ v ][ 2 ] = ( it->z - centerZ ) * scale;
		}

		// ���_�@���̒l���O�ɂ��Ă���
		for ( GLuint i = 0; i < vertexNum; ++i ){
			norm[ i ][ 0 ] = norm[ i ][ 1 ] = norm[ i ][ 2 ] = 0.0f;
		}

		// �ʂ̖@���̎Z�o�ƃf�[�^�̃R�s�[
		for ( std::vector<index>::const_iterator it = tface.begin(); it != tface.end(); ++it ){
			const size_t f( it - tface.begin() );

			// ���_���W�ԍ������o��
			const GLuint v0( face[ f ][ 0 ] = it->position[ 0 ] - 1 );
			const GLuint v1( face[ f ][ 1 ] = it->position[ 1 ] - 1 );
			const GLuint v2( face[ f ][ 2 ] = it->position[ 2 ] - 1 );

			// v1 - v0, v2 - v0 �����߂�
			const GLfloat d1[] = {
				pos[ v1 ][ 0 ] - pos[ v0 ][ 0 ],
				pos[ v1 ][ 1 ] - pos[ v0 ][ 1 ],
				pos[ v1 ][ 2 ] - pos[ v0 ][ 2 ]
			};
			const GLfloat d2[] = {
				pos[ v2 ][ 0 ] - pos[ v0 ][ 0 ],
				pos[ v2 ][ 1 ] - pos[ v0 ][ 1 ],
				pos[ v2 ][ 2 ] - pos[ v0 ][ 2 ]
			};

			// �O�ςɂ��ʖ@�������߂�
			GLfloat n[ 3 ];
			cross( n, d1, d2 );

			// �ʖ@���𒸓_�@���ɐώZ����
			norm[ v0 ][ 0 ] += n[ 0 ];
			norm[ v0 ][ 1 ] += n[ 1 ];
			norm[ v0 ][ 2 ] += n[ 2 ];
			norm[ v1 ][ 0 ] += n[ 0 ];
			norm[ v1 ][ 1 ] += n[ 1 ];
			norm[ v1 ][ 2 ] += n[ 2 ];
			norm[ v2 ][ 0 ] += n[ 0 ];
			norm[ v2 ][ 1 ] += n[ 1 ];
			norm[ v2 ][ 2 ] += n[ 2 ];
		}

		// ���_�@���̐��K��
		for ( GLuint v = 0; v < vertexNum; ++v ){
			// ���_�@���̒���
			GLfloat a( sqrt( dot3( norm[ v ], norm[ v ] ) ) );

			// ���_�@���̐��K��
			if ( a != 0.0f ){
				norm[ v ][ 0 ] /= a;
				norm[ v ][ 1 ] /= a;
				norm[ v ][ 2 ] /= a;
			}
		}

		return true;
	}

	
}