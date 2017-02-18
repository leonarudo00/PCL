// C��min,max�}�N���𖳌��ɂ���
#define NOMINMAX
#define _CRT_SECURE_NO_WARNINGS

#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "glfw3.lib")
#pragma comment(lib, "glew32.lib")

#include <iostream>
#include <Windows.h>
#include <pcl\visualization\cloud_viewer.h>
#include <pcl\io\vtk_lib_io.h>

#include <cstdlib>
#include <vector>
#include <memory>
#include <fstream>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "window.h"
#include "Shape.h"
#include "MyOpenGL.h"
#include "Mesh.h"

// obj�f�[�^���擾
const char filename[] = "bunny.obj";

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

// ��`�̒��_�̈ʒu
const Object::Vertex rectangleVertex[]=
{
	{ -0.5f, -0.5f },
	{  0.5f, -0.5f },
	{  0.5f,  0.5f },
	{ -0.5f,  0.5 }
};

GLfloat projectionMatrix[ 16 ];		// ���e�ϊ��s��
GLfloat temp0[ 16 ], temp1[ 16 ];	// �ꎞ�I�ȕϊ��s��

void main()
{
	// ����w�b�_
	MyOpenGL myOpenGL;

	// GLFW ������������
	if ( glfwInit() == GL_FALSE ){
		// �������Ɏ��s����
		std::cerr << "Can't initialize GLFW" << std::endl;
		return;
	}

	// �v���O�����I�����̏�����o�^����
	atexit( glfwTerminate );

	// OpenGL Version 3.2 Core Profile ��I������
	glfwWindowHint( GLFW_CONTEXT_VERSION_MAJOR, 3 );
	glfwWindowHint( GLFW_CONTEXT_VERSION_MINOR, 2 );
	glfwWindowHint( GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE );
	glfwWindowHint( GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE );

	// �E�B���h�E���쐬����
	Window window;

	// �w�i�F���w�肷��
	glClearColor( 0.0f, 0.0f, 0.3f, 0.0f );

	// �v���O�����I�u�W�F�N�g���쐬����
	const GLuint program( loadProgram( "point.vert", "point.frag" ) );

	// ���s���e�ϊ��s������߂�
	//myOpenGL.orthogonalMatrix( -1.0f, 1.0f, -1.0f, 1.0f, 7.0f, 11.0f, temp1 );
	//myOpenGL.perspectiveMatrix( -1.0f, 1.0f, -1.0f, 1.0f, 7.0f, 11.0f, temp1 );
	myOpenGL.cameraMatrix( 30.f, 1.0f, 7.0f, 11.0f, temp1 );

	// ����ϊ��s������߂�
	myOpenGL.lookAt( 4.0f, 5.0f, 6.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, temp0 );
	// ����ϊ��s��Ɠ��e�ϊ��s��̐ς����߂�
	myOpenGL.multiplyMatrix( temp0, temp1, projectionMatrix );

	// uniform�ϐ��̏ꏊ���擾����
	const GLint sizeLoc( glGetUniformLocation( program, "size" ) );
	const GLint scaleLoc( glGetUniformLocation( program, "scale" ) );
	const GLint locationLoc( glGetUniformLocation( program, "location" ) );
	const GLint projectionMatrixLoc( glGetUniformLocation( program, "projectionMatrix" ) );

	// �}�`�f�[�^���쐬����
	std::unique_ptr<const Shape> shape( new Shape( 2, 4, rectangleVertex ) );
	Mesh mesh( 16, 12 );

	// �E�B���h�E���J���Ă���ԌJ��Ԃ�
	while ( window.shouldClose() == GL_FALSE ){
		// �E�B���h�E����������
		glClear( GL_COLOR_BUFFER_BIT );

		// �V�F�[�_�v���O�����̎g�p�J�n
		glUseProgram( program );

		// uniform�ϐ��ɒl��ݒ肷��
		glUniform2fv( sizeLoc, 1, window.getSize() );
		glUniform1f( scaleLoc, window.getScale() );
		glUniform2fv( locationLoc, 1, window.getLocation() );
		glUniformMatrix4fv( projectionMatrixLoc, 1, GL_FALSE, projectionMatrix );

		// �}�`��`�悷��
		//shape->draw();
		mesh.draw();


		// �J���[�o�b�t�@�����ւ��ăC�x���g�����o��
		window.swapBuffers();
	}


	try{
		// obj�t�@�C����ǂݍ���
		pcl::PolygonMesh::Ptr mesh( new pcl::PolygonMesh() );
		pcl::PointCloud<pcl::PointXYZ>::Ptr obj_pcd( new pcl::PointCloud<pcl::PointXYZ>() );
		if ( pcl::io::loadPolygonFileOBJ( filename, *mesh ) != -1 ){
			pcl::fromPCLPointCloud2( mesh->cloud, *obj_pcd );
		}

		// �E�B���h�E�̍쐬
		pcl::visualization::PCLVisualizer viewer( "Point Cloud Viewer" );

		// PointCloud��ǉ�
		viewer.addPointCloud( obj_pcd );

		// �E�B���h�E�����Ă��Ȃ��ԑ���
		while ( !viewer.wasStopped() ) {
			// �X�N���[�����X�V
			viewer.spinOnce();

			// ESCAPE�L�[�������ꂽ��I��
			if ( GetKeyState( VK_ESCAPE ) < 0 ){
				break;
			}
		}
	}
	catch ( std::exception& ex ){
		std::cout << ex.what() << std::endl;
	}
}