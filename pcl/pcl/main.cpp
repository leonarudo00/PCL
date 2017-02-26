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
	const GLuint program( MyOpenGL::loadProgram( "point.vert", "point.frag" ) );

	// ���s���e�ϊ��s������߂�
	//myOpenGL.orthogonalMatrix( -1.0f, 1.0f, -1.0f, 1.0f, 7.0f, 11.0f, temp1 );
	//myOpenGL.perspectiveMatrix( -1.0f, 1.0f, -1.0f, 1.0f, 7.0f, 11.0f, temp1 );
	MyOpenGL::cameraMatrix( 30.f, 1.0f, 7.0f, 11.0f, temp1 );

	// ����ϊ��s������߂�
	MyOpenGL::lookAt( 4.0f, 5.0f, 6.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, temp0 );
	// ����ϊ��s��Ɠ��e�ϊ��s��̐ς����߂�
	MyOpenGL::multiplyMatrix( temp0, temp1, projectionMatrix );

	// uniform�ϐ��̏ꏊ���擾����
	const GLint sizeLoc( glGetUniformLocation( program, "size" ) );
	const GLint scaleLoc( glGetUniformLocation( program, "scale" ) );
	const GLint locationLoc( glGetUniformLocation( program, "location" ) );
	const GLint projectionMatrixLoc( glGetUniformLocation( program, "projectionMatrix" ) );

	// �}�`�f�[�^���쐬����
	std::unique_ptr<const Shape> shape( new Shape( 2, 4, rectangleVertex ) );
	//Mesh mesh( 16, 12 );
	Mesh mesh( filename, false );

	// �B�ʏ���������L���ɂ���
	glEnable( GL_DEPTH_TEST );

	// �E�B���h�E���J���Ă���ԌJ��Ԃ�
	while ( window.shouldClose() == GL_FALSE ){
		// �E�B���h�E����������
		glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

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