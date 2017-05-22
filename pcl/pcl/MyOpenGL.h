#pragma once
#define pi	3.14159265f	// �~����
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
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
		glBindAttribLocation( program, 1, "normal" );
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

	//
	// ���s���e�ϊ��s������߂�
	//
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

	//
	// �������e�ϊ��s������߂�
	//
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

	//
	// ����ϊ��s������߂�
	//
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

	//
	// �s��̐ς����߂�
	//
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

	//
	// ��p���瓧�����e�ϊ��s������߂�
	//
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

	//
	// �ϊ��s��F�@���ϊ��s���ݒ肷��
	//
	void loadNormal( GLfloat *marray, GLfloat *array )
	{
		array[ 0 ]  = marray[ 5 ]  * marray[ 10 ] - marray[ 6 ]  * marray[ 9 ];
		array[ 1 ]  = marray[ 6 ]  * marray[ 8 ]  - marray[ 4 ]  * marray[ 10 ];
		array[ 2 ]  = marray[ 4 ]  * marray[ 9 ]  - marray[ 5 ]  * marray[ 8 ];
		array[ 4 ]  = marray[ 9 ]  * marray[ 2 ]  - marray[ 10 ] * marray[ 1 ];
		array[ 5 ]  = marray[ 10 ] * marray[ 0 ]  - marray[ 8 ]  * marray[ 2 ];
		array[ 6 ]  = marray[ 8 ]  * marray[ 1 ]  - marray[ 9 ]  * marray[ 0 ];
		array[ 8 ]  = marray[ 1 ]  * marray[ 6 ]  - marray[ 2 ]  * marray[ 5 ];
		array[ 9 ]  = marray[ 2 ]  * marray[ 4 ]  - marray[ 0 ]  * marray[ 6 ];
		array[ 10 ] = marray[ 0 ]  * marray[ 5 ]  - marray[ 1 ]  * marray[ 4 ];
		
		array[ 3 ]  = array[ 7 ] = array[ 11 ] = array[ 12 ] = array[ 13 ] = array[ 14 ] = 0.0f;
		array[ 15 ] = 1.0f;

		return;
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

	// TGA�t�@�C����ǂݍ���
	// name:	�ǂݍ��ރt�@�C����
	// width:	�ǂݍ��񂾃t�@�C���̕�
	// height:	�ǂݍ��񂾃t�@�C���̍���
	// format:	�ǂݍ��񂾃t�@�C���̃t�H�[�}�b�g
	// return:	�ǂݍ��񂾉摜�f�[�^�̃|�C���^�B�ǂݍ��߂Ȃ����nullptr
	GLubyte *loadTga( const char *name, GLsizei *width, GLsizei *height, GLenum *format )
	{
		// �t�@�C�����J��
		std::ifstream file( name, std::ios::binary );

		// �t�@�C�����J���Ȃ���������ǂ�
		if ( !file )
		{
			std::cerr << "Error: Can't open file: " << name << std::endl;
			return nullptr;
		}

		// �w�b�_��ǂݍ���
		unsigned char header[ 18 ];
		file.read( reinterpret_cast< char* >( header ), sizeof header );

		// �w�b�_�̓ǂݍ��݂Ɏ��s��������ǂ�
		if ( file.bad() )
		{
			std::cerr << "Error: Can't read file header: " << name << std::endl;
			file.close();
			return nullptr;
		}

		// ���ƍ���
		*width = header[ 13 ] << 8 | header[ 12 ];
		*height = header[ 15 ] << 8 | header[ 14 ];

		// �[�x
		const size_t depth( header[ 16 ] / 8 );
		switch ( depth )
		{
			case 1:
				*format = GL_RED;
				break;
			case 2:
				*format = GL_RG;
				break;
			case 3:
				*format = GL_BGR;
				break;
			case 4:
				*format = GL_BGRA;
				break;
			default:
				// ���݂��Ȃ��t�H�[�}�b�g�Ȃ�߂�
				std::cerr << "Error: Unusable format: " << depth << std::endl;
				file.close();
				return nullptr;
		}

		// �f�[�^�T�C�Y
		const size_t size( *width * *height * depth );

		// �ǂݍ��݂Ɏg�����������m�ۂ���
		GLubyte *const buffer( new( std::nothrow )GLubyte[ size ] );

		// ���������m�ۂł��Ȃ���΂��ǂ�
		if ( buffer == nullptr )
		{
			std::cerr << "Error: Too large file: " << name << std::endl;
			file.close();
			return nullptr;
		}

		// �f�[�^��ǂݍ���
		if ( header[ 2 ] & 8 )
		{
			// RLE
			size_t p( 0 );
			char c;
			while ( file.get( c ) )
			{
				if ( c & 0x80 )
				{
					// run-length packet
					const size_t count( ( c & 0x7f ) + 1 );
					if ( p + count * depth > size )break;
					char tmp[ 4 ];
					file.read( tmp, depth );
					for ( size_t i = 0; i < count; ++i )
					{
						for ( size_t j = 0; j < depth; ) buffer[ p++ ] = tmp[ j++ ];
					}
				}
				else
				{
					// raw packet
					const size_t count( ( c + 1 ) * depth );
					if ( p + count > size )break;
					file.read( reinterpret_cast< char * >( buffer + p ), count );
					p += count;
				}
			}
		}
		else
		{
			// �񈳏k
			file.read( reinterpret_cast< char * >( buffer ), size );
		}

		// �ǂݍ��݂Ɏ��s���Ă�����x�����o��
		if ( file.bad() )
		{
			std::cerr << "Warning: Can't read image data: " << name << std::endl;
		}

		// �t�@�C�������
		file.close();

		// �摜��ǂݍ��񂾃�������Ԃ�
		return buffer;
	}

	// �z��Ɋi�[���ꂽ�摜�̂Ȃ��悤��TGA�t�@�C���ɕۑ�����
	// width:	�摜�̕�
	// height:	�摜�̍���
	// depth:	�摜�̂P��f�̃o�C�g��
	// buffer:	�摜�f�[�^
	// name:	�t�@�C����
	// return:	�ۑ��ɐ���������
	bool saveTga( GLsizei width, GLsizei height, unsigned int depth, const void *buffer, const char *name )
	{
		// �t�@�C�����J��
		std::ofstream file( name, std::ios::binary );

		// �t�@�C�����J���Ȃ���������ǂ�
		if ( !file )
		{
			std::cerr << "Error: Can't open file: " << name << std::endl;
			return false;
		}

		// �摜�̃w�b�_
		const unsigned char type( depth == 0 ? 0 : depth < 3 ? 3 : 2 );
		const unsigned char alpha( depth == 2 || depth == 4 ? 8 : 0 );
		const unsigned char header[ 18 ] =
		{
			0,		// ID length
			0,		// Color map type (none)
			type,	// Image Type (2:RGB, 3:Grayscale)
			0, 0,	// Offset into the color map table
			0, 0,	// Number of color map entries
			0,		// Number of a color map entry bits per pixel
			0, 0,	// Horizontal image position
			0, 0,	// Vertical image position
			( unsigned char )( width & 0xff ),
			( unsigned char )( width >> 8 ),
			( unsigned char )( height & 0xff ),
			( unsigned char )( height >> 8 ),
			( unsigned char )( depth * 8 ),	// Pixel depth(bits per pixel)
			alpha	// Image descriptor
		};

		// �w�b�_����������
		file.write( reinterpret_cast< const char* >( header ), sizeof header );

		// �w�b�_�̏������݃`�F�b�N
		if ( file.bad() )
		{
			// �w�b�_�̏������݂Ɏ��s����
			std::cerr << "Error: Can't write file header: " << name << std::endl;
			file.close();
			return false;
		}

		// �f�[�^����������
		size_t size( width * height * depth );
		if ( type == 2 )
		{
			// �t���J���[
			std::vector<char> temp( size );
			for ( size_t i = 0; i < size; i += depth )
			{
				temp[ i + 2 ] = static_cast< const char* >( buffer )[ i + 0 ];
				temp[ i + 1 ] = static_cast< const char* >( buffer )[ i + 1 ];
				temp[ i + 0 ] = static_cast< const char* >( buffer )[ i + 2 ];
				if ( depth == 4 )temp[ i + 3 ] = static_cast< const char* >( buffer )[ i + 3 ];
			}
			file.write( &temp[ 0 ], size );
		}
		else if ( type == 3 )
		{
			// �O���[�X�P�[��
			file.write( static_cast< const char* >( buffer ), size );
		}

		// �t�b�^����������
		static const char footer[] = "\0\0\0\0\0\0\0\0TRUEVISION-XFILE.";
		file.write( footer, sizeof footer );

		// �f�[�^�̏������݃`�F�b�N
		if ( file.bad() )
		{
			// �f�[�^�̏������݂Ɏ��s����
			std::cerr << "Error: Can't write image data: " << name << std::endl;
			file.close();
			return false;
		}

		// �t�@�C�������
		file.close();

		return true;
	}

	//
	// ������
	//
	void smooth( const GLubyte *src, GLsizei width, GLsizei height, GLenum format, GLsizei xc,
		GLsizei yc, GLsizei xr, GLsizei yr, GLubyte *dst, GLsizei size, const GLfloat *amb, GLfloat shi )
	{
		// �`�����l����
		const int channels( format == GL_BGRA ? 4 : 3 );

		// ���������x
		const GLfloat ramb( amb[ 0 ] * 255.0f ), gamb( amb[ 1 ] * 255.0f ), bamb( amb[ 2 ] * 255.0f );

		// ���ˏƓx�}�b�v�̊e��f�ɂ���
		for ( int yd = 0; yd < size; ++yd )
		{
			std::cout << "Processing line: " << yd
				<< "(" << std::fixed << std::setprecision( 1 ) << float( yd )*100.0f / float( size ) << "%)"
				<< std::endl;

			for ( int xd = 0; xd < size; ++xd )
			{
				// ���̉�f�̕��ˏƓx�}�b�v�̔z��dst�̃C���f�b�N�X
				const int id( ( yd * size + xd ) * 3 );

				// ���̉�f�̕��ˏƓx�}�b�v��̐��K�����ꂽ���W�l�i-0.5 <= u, v >= 0.5�j
				const float u( float( xd ) / float( size - 1 ) - 0.5f );
				const float v( 0.5f - float( yd ) / float( size - 1 ) );
				const float m( u * u + v * v );
				const float w( 0.25f - m );
				const float a( sqrt( m + w * w ) );

				// ���ˏƓx�}�b�v������ʃ}�b�v�Ƃ��ĎQ�Ƃ���Ƃ��̂��̉�f�̕����x�N�g��
				const float qx( u / a );
				const float qy( w / a );
				const float qz( v / a );

				// ���̉�f�����ˏƓx�}�b�v�̒P�ʉ~�O�ɂ���Ƃ�
				if ( qy <= 0.0f )
				{
					// ��������ݒ肷��
					dst[ id + 0 ] = GLubyte( ramb );
					dst[ id + 1 ] = GLubyte( gamb );
					dst[ id + 2 ] = GLubyte( bamb );
					continue;
				}

				// ���̃x�N�g���̕�����V���Ƃ��锼�V������̕��ˏƓx�̑��a
				float rsum( 0.0f ), gsum( 0.0f ), bsum( 0.0f );

				// ���V���̏d�݂Â����̊p�̑��a
				float wtotal( 0.0f );

				// src��(xc, yc)�𒆐S�Ƃ�[-xr, xr] x [-yr, yr]�͈̔͂̊e��f�ɂ���
				for ( int ys = yc - yr; ys <= yc + yr; ++ys )
				{
					for ( int xs = xc - xr; xs <= xc + xr; ++xs )
					{
						// ���̉�f�̓V��摜��̐��K�����ꂽ���W�l(-1 <= s, t <= 1)
						const float s( float( xs - xc ) / float( xr ) );
						const float t( float( yc - ys ) / float( yr ) );

						// ���̉�f�̓V��摜�̒��S����̋���
						const float r( sqrt( s * s + t * t ) );

						// ���̉摜�̓V��摜�̒��S����̋�����V���p�Ƃ�������x�N�g��p��y����
						const float py( cos( r * float( pi ) * 0.5f ) );

						// ���̉�f�̓V��摜�̒��S����̋����ɑ΂�������x�N�g��q��xz�����̒����̔�
						const float l( r > 0.0f ? sqrt( 1.0f - py * py ) / r : 0.0f );

						// ���̉�f�̓V��Ɍ����������x�N�g��q��x������x����
						const float px( s * l );
						const float pz( t * l );

						// p��q�̓���
						const float pq( px * qx + py * qy + pz * qz );

						// ���̉�f�̕���p�����ˏƓx�}�b�v�̕����x�N�g��q�̔��Α��������Ă���Ƃ�
						if ( pq <= 0.0f )continue;

						// ���̉�f�̕���p�̓V���p
						const float theta( acos( pq ) );

						// ���̉�f�̕���p�̗��̊p(��(1 - cos��^2) / �� = sin�� / �� = sinc��)
						const float sr( theta > 0.0f ? sqrt( 1.0f - pq * pq ) / theta : 1.0f );

						// ���̉�f�̕���p�̗��̊p��shininess�̏d�݂�����
						const float dw( pow( pq, shi ) * sr );

						// �d�݂Â����̊p��ώZ����
						wtotal += dw;

						// ���̉�f���V��摜�̒P�ʉ~�O�ɂ���Ƃ�
						if ( r >= 1.0f )
						{
							// �����������Z����
							rsum += ramb * dw;
							gsum += gamb * dw;
							bsum += bamb * dw;
							continue;
						}

						// ���̉�f�̓V��摜�̔z��src�̃C���f�b�N�X
						const int is( ( ys * width + xs ) * channels );

						// src�̉�f�l��dst�ɉ��Z����
						rsum += float( src[ is + 2 ] ) * dw;
						gsum += float( src[ is + 1 ] ) * dw;
						bsum += float( src[ is + 0 ] ) * dw;
					}
				}

				// �d�ݕt�����̊p�̑��a�i�V��̖ʐρj�Ŋ���
				dst[ id + 0 ] = GLubyte( round( rsum / wtotal ) );
				dst[ id + 1 ] = GLubyte( round( gsum / wtotal ) );
				dst[ id + 2 ] = GLubyte( round( bsum / wtotal ) );
			}
		}
	}

	//
	// �e�N�X�`���̍쐬
	//
	void createTexture( const GLubyte *buffer, GLsizei width, GLsizei height, GLenum format,
		const GLfloat *amb, GLuint tex )
	{
		// �e�N�X�`���I�u�W�F�N�g�Ƀe�N�X�`�������蓖�Ă�
		glBindTexture( GL_TEXTURE_2D, tex );
		glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, format, GL_UNSIGNED_BYTE, buffer );

		// �e�N�X�`���͐��`��Ԃ���
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );

		// ���E�����g������
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER );

		// �e�N�X�`���̋��E�F�ɑ�������ݒ肷��
		glTexParameterfv( GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, amb );
	}
	//
	// �e�N�X�`���̍쐬�i�~�b�v�}�b�v�j
	//
	GLuint createTexture( GLenum internalFormat, GLsizei width, GLsizei height, GLint levels )
	{
		// �e�N�X�`���̋��E�F
		const GLfloat border[] = { 0.0f, 0.0f, 0.0f, 0.0f };

		// �e�N�X�`���I�u�W�F�N�g���쐬����
		GLuint texture;
		glGenTextures( 1, &texture );

		// �e�N�X�`���I�u�W�F�N�g�Ƀe�N�X�`�������蓖�Ă�
		glBindTexture( GL_TEXTURE_2D, texture );
		for ( GLint level = 0; level <= levels; ++level )
		{
			glTexImage2D( GL_TEXTURE_2D, level, internalFormat, width, height, 0, GL_BGR, GL_UNSIGNED_BYTE, NULL );
			width = std::max( 1, ( width / 2 ) );
			height = std::max( 1, ( height / 2 ) );
		}

		// �e�N�X�`���͐��`��Ԃ���
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR );

		// ���E�����g������
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER );

		// ���E�F��ݒ肷��
		glTexParameterfv( GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border );

		return texture;
	}

	//
	// ���ˏƓx�}�b�v�̍쐬
	//
	bool createMap( const char *name, GLsizei diameter, GLuint imap, GLsizei isize, 
		GLuint emap, GLsizei esize, const GLfloat *amb, GLfloat shi )
	{
		// �쐬�����e�N�X�`���̐�
		static int count( 0 );

		// �ǂݍ��񂾉摜�̕��ƍ����A�t�H�[�}�b�g
		GLsizei width, height;
		GLenum format;

		// �V��摜�̃t�@�C���̓ǂݍ���
		GLubyte const *const texture( loadTga( name, &width, &height, &format ) );

		// �摜���ǂݍ��߂Ȃ���ΏI��
		if ( !texture )return false;

		// ���̉摜�̒��S�ʒu
		const GLsizei centerX( width / 2 ), centerY( height / 2 );

		// diameter, width, height�̍ŏ��l��1/2��raduis�ɂ���
		const GLsizei radius( std::min( diameter, std::min( width, height ) ) / 2 );

		// �������������ˏƓx�}�b�v�̈ꎞ�ۑ���
		std::vector<GLubyte> itemp( isize * isize * 3 );

		// ���ˏƓx�}�b�v�p�ɕ�������
		smooth( texture, width, height, format, centerX, centerY, radius, radius, &itemp[ 0 ], isize, amb, 1.0f );

		// ���ˏƓx�}�b�v�̃e�N�X�`�����쐬����
		createTexture( &itemp[ 0 ], isize, isize, GL_RGB, amb, imap );

		// �쐬�����e�N�X�`����ۑ�����
		std::stringstream imapname;
		imapname << "irr" << count << ".tga";
		saveTga( isize, isize, 3, &itemp[ 0 ], imapname.str().c_str() );

		// �����������}�b�v�̈ꎞ�ۑ���
		std::vector<GLubyte> etemp( esize * esize * 3 );

		// ���}�b�v�p�ɕ�������
		smooth( texture, width, height, format, centerX, centerY, radius, radius, &etemp[ 0 ], esize, amb, shi );

		// ���}�b�v�̃e�N�X�`�����쐬����
		createTexture( &etemp[ 0 ], esize, esize, GL_RGB, amb, emap );

		// ���}�b�v�̃e�N�X�`�����쐬����
		std::stringstream emapname;
		emapname << "env" << count << ".tga";
		saveTga( esize, esize, 3, &etemp[ 0 ], emapname.str().c_str() );

		// �ǂݍ��񂾃f�[�^�͂����g��Ȃ��̂Ń��������J������
		delete[] texture;

		// �쐬�����e�N�X�`���̐��𐔂���
		++count;

		return true;
	}

	//
	// ���ˏƓx�}�b�v�̓ǂݍ���
	//
	bool loadMap( const char *iname, const char *ename, GLuint imap, GLuint emap )
	{
		// �I���X�e�[�^�X
		bool status( true );

		// �ǂݍ��񂾉摜�̕��ƍ����A�t�H�[�}�b�g
		GLsizei width, height;
		GLenum format;

		// ���ˏƓx�}�b�v�̓ǂݍ���
		GLubyte const *const itexture( loadTga( iname, &width, &height, &format ) );

		// �摜���ǂݍ��߂Ȃ���ΏI��
		if ( !itexture )status = false;

		// �ǂݍ��񂾉摜�̍�����̉�f�̐F��������Ƃ���
		const GLfloat iamb[] = { itexture[ 2 ] / 255.0f, itexture[ 1 ] / 255.0f, itexture[ 0 ] / 255.0f };

		// �e�N�X�`���̍쐬
		createTexture( itexture, width, height, format, iamb, imap );

		// �ǂݍ��񂾃f�[�^�͂����g��Ȃ��̂Ń��������������
		delete[] itexture;

		// ���ˏƓx�}�b�v�̓ǂݍ���
		GLubyte const *const etexture( loadTga( ename, &width, &height, &format ) );

		// �摜���ǂݍ��߂Ȃ���ΏI��
		if ( !etexture )status = false;

		// �ǂݍ��񂾉摜�̍�����̉�f�̐F��������Ƃ���
		const GLfloat eamb[] = { etexture[ 2 ] / 255.0f, etexture[ 1 ] / 255.0f, etexture[ 0 ] / 255.0f };

		// �e�N�X�`���̍쐬
		createTexture( etexture, width, height, format, eamb, emap );

		// �ǂݍ��񂾃f�[�^�͂����g��Ȃ��̂Ń��������J������
		delete[] etexture;

		// �S�ēǂݍ��ݐ���
		return status;
	}

	//
	// �����ʃ}�b�s���O�p�̃e�N�X�`���ϊ��s��
	//
	const GLfloat paraboloid[] =
	{
		-1.0f, 0.0f, 0.0f, 0.0f,
		 1.0f, 1.0f, 0.0f, 2.0f,
		 0.0f,-1.0f, 0.0f, 0.0f,
		 1.0f, 1.0f, 0.0f, 2.0f,
	};

	//
	// ���ˏƓx�}�b�v�Ɏg���e�N�X�`�����j�b�g�̐ݒ�
	//
	void irradiance()
	{
		// �e�N�X�`�����W�ɖ@���x�N�g�����g��
		glTexGeni( GL_S, GL_TEXTURE_GEN_MODE, GL_NORMAL_MAP );
		glTexGeni( GL_T, GL_TEXTURE_GEN_MODE, GL_NORMAL_MAP );
		glTexGeni( GL_R, GL_TEXTURE_GEN_MODE, GL_NORMAL_MAP );
		glEnable( GL_TEXTURE_GEN_S );
		glEnable( GL_TEXTURE_GEN_T );
		glEnable( GL_TEXTURE_GEN_R );

		// ���ˏƓx�}�b�v�̒l�������グ���� Ce = Cb + Ct
		glTexEnvi( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_COMBINE );
		glTexEnvi( GL_TEXTURE_ENV, GL_COMBINE_RGB, GL_ADD );			// ���Z
		glTexEnvi( GL_TEXTURE_ENV, GL_SOURCE0_RGB, GL_CONSTANT );
		glTexEnvi( GL_TEXTURE_ENV, GL_OPERAND0_RGB, GL_SRC_COLOR );		// Cb:GL_TEXTURE_ENV_COLOR��RGB�l
		glTexEnvi( GL_TEXTURE_ENV, GL_SOURCE1_RGB, GL_TEXTURE );
		glTexEnvi( GL_TEXTURE_ENV, GL_OPERAND1_RGB, GL_SRC_COLOR );		// Ct:���ˏƓx�}�b�v�̒l

		// �e�N�X�`�����W�̕ϊ��s��ɕ����ʃ}�b�s���O�p�̕ϊ��s���ݒ肷��
		glMatrixMode( GL_TEXTURE );
		glLoadMatrixf( paraboloid );
	}

	//
	// �g�U���ˌ����x�̎Z�o�Ɏg���e�N�X�`�����j�b�g�̐ݒ�
	//
	void diffuse()
	{
		// ���̂̐F�i���_�F�̕�Ԓl�j�ɑO���C���ŋ��߂����ˌ����x�������� Cd = Cv * Ce
		glTexEnvi( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_COMBINE );
		glTexEnvi( GL_TEXTURE_ENV, GL_COMBINE_RGB, GL_MODULATE );		// ��Z
		glTexEnvi( GL_TEXTURE_ENV, GL_SOURCE0_RGB, GL_PRIMARY_COLOR );
		glTexEnvi( GL_TEXTURE_ENV, GL_OPERAND0_RGB, GL_SRC_COLOR );		// Cv:���_�F�̕�Ԓl
		glTexEnvi( GL_TEXTURE_ENV, GL_SOURCE1_RGB, GL_PREVIOUS );
		glTexEnvi( GL_TEXTURE_ENV, GL_OPERAND1_RGB, GL_SRC_COLOR );		// Ce:�����グ�������ˏƓx�i�O���C���j
	}

	//
	// ���}�b�v�̉��Z�Ɏg���e�N�X�`�����j�b�g�̐ݒ�
	//
	void reflection()
	{
		// �e�N�X�`�����W�ɔ��˃x�N�g�����g��
		glTexGeni( GL_S, GL_TEXTURE_GEN_MODE, GL_REFLECTION_MAP );
		glTexGeni( GL_T, GL_TEXTURE_GEN_MODE, GL_REFLECTION_MAP );
		glTexGeni( GL_R, GL_TEXTURE_GEN_MODE, GL_REFLECTION_MAP );
		glEnable( GL_TEXTURE_GEN_S );
		glEnable( GL_TEXTURE_GEN_T );
		glEnable( GL_TEXTURE_GEN_R );

		// ���}�b�v�̒l�ƑO���C���ł��Ƃ߂��g�U���ˌ����x�����ʔ��ˌW���Ŕ��z������ C = Ct * Cs + Cd * ( 1 - Cs )
		glTexEnvi( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_COMBINE );
		glTexEnvi( GL_TEXTURE_ENV, GL_COMBINE_RGB, GL_INTERPOLATE );	// ���
		glTexEnvi( GL_TEXTURE_ENV, GL_SOURCE0_RGB, GL_TEXTURE );
		glTexEnvi( GL_TEXTURE_ENV, GL_OPERAND0_RGB, GL_SRC_COLOR );		// Ct:���}�b�v�̒l
		glTexEnvi( GL_TEXTURE_ENV, GL_SOURCE1_RGB, GL_PREVIOUS );
		glTexEnvi( GL_TEXTURE_ENV, GL_OPERAND1_RGB, GL_SRC_COLOR );		// Cd:�g�U���ˌ����x�i�O���C���j
		glTexEnvi( GL_TEXTURE_ENV, GL_SOURCE2_RGB, GL_CONSTANT );
		glTexEnvi( GL_TEXTURE_ENV, GL_OPERAND2_RGB, GL_SRC_COLOR );		// Cs:���ʔ��ˌW��

		// �e�N�X�`�����W�̕ϊ��s��ɕ����ʃ}�b�s���O�p�̕ϊ��s���ݒ肷��
		glMatrixMode( GL_TEXTURE );
		glLoadMatrixf( paraboloid );
	}

	void setupTexture( GLuint texID, const char *file, const int width, const int height, const GLenum format )
	{
		// Step2. �摜�f�[�^�̃��[�h
		std::ifstream fstr( file, std::ios::binary );
		assert( fstr );

		const size_t fileSize = static_cast<size_t>( fstr.seekg( 0, fstr.end ).tellg() );
		fstr.seekg( 0, fstr.beg );
		std::vector<char> textureBuffer( fileSize );
		fstr.read( &textureBuffer[ 0 ], fileSize );

		// Step3. �摜�f�[�^�ƃe�N�X�`��iD�����т���
		glBindTexture( GL_TEXTURE_2D, texID );
		glTexImage2D( GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, &textureBuffer[ 0 ] );

		// Step4. �e�N�X�`���̊e��ݒ�
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );
	}
}