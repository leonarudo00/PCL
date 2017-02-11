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
#include <GL/glew.h>
#include <GLFW/glfw3.h>


// obj�f�[�^���擾
const char filename[] = "bunny.obj";

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
	GLFWwindow *const window( glfwCreateWindow( 640, 480, "Hello!", NULL, NULL ) );
	if ( window == NULL )
	{
		// �E�B���h�E���쐬�ł��Ȃ�����
		std::cerr << "Can't create GLFW window." << std::endl;
		glfwTerminate();
		return;
	}

	// �쐬�����E�B���h�E�� OpenGL �̏����Ώۂɂ���
	glfwMakeContextCurrent( window );

	// GLEW ������������
	glewExperimental = GL_TRUE;
	if ( glewInit() != GLEW_OK ){
		// GLEW �̏������Ɏ��s����
		std::cerr << "Can't initialize GLEW" << std::endl;
		return;
	}

	// ���������̃^�C�~���O��҂�
	glfwSwapInterval( 1 );

	// �w�i�F���w�肷��
	glClearColor( 1.0f, 1.0f, 1.0f, 0.0f );

	// �E�B���h�E���J���Ă���ԌJ��Ԃ�
	while ( glfwWindowShouldClose( window ) == GL_FALSE ){
		// �E�B���h�E����������
		glClear( GL_COLOR_BUFFER_BIT );

		//
		// �����ŕ`�揈�����s��
		//

		// �J���[�o�b�t�@�����ւ���
		glfwSwapBuffers( window );
		// �C�x���g�����o��
		glfwWaitEvents();
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