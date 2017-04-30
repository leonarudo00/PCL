// C��min,max�}�N���𖳌��ɂ���
#define NOMINMAX
#define _CRT_SECURE_NO_WARNINGS

#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "glfw3.lib")
#pragma comment(lib, "glew32.lib")

#include <iostream>
#include <Windows.h>
//#include <pcl\visualization\cloud_viewer.h>
//#include <pcl\point_types.h> 
//#include <pcl\io\vtk_lib_io.h>
#include <cstdlib>
#include <vector>
#include <memory>
#include <fstream>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "window.h"
#include "MyOpenGL.h"
#include "Mesh.h"
#include "opencv2\opencv.hpp"

// �L���v�`���ɗp����J�����̃f�o�C�X�ԍ�
const int captureDevice( 1 );

// ���O�v�Z�������ˏƓx�}�b�v���g�p����Ȃ� 1
#define USEMAP 1

// obj�f�[�^���擾
const char filename[] = "mario.obj";

// ���ˏƓx�}�b�v
const char *const irrmaps[]=
{
	"irr0.tga"
};

// ���}�b�v
const char *const envmaps[]=
{
	"env0.tga"
};

// ���ˏƓx�}�b�v�̐�
const size_t mapcount( sizeof irrmaps / sizeof irrmaps[ 0 ] );

// �V��摜
const char skymap[] = "skymap0.tga";

// ���������x
const GLfloat ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };

// �P���W��
const GLfloat shininess( 60.0f );

// �V��摜���̓V��̈�̒��a�̍ő�l
const GLsizei skysize( 1024 );

// �쐬����e�N�X�`���̃T�C�Y
const GLsizei imapsize( 256 );
const GLsizei emapsize( 256 );

GLfloat projectionMatrix[ 16 ];		// ���e�ϊ��s��
GLfloat temp0[ 16 ], temp1[ 16 ];	// �ꎞ�I�ȕϊ��s��

// �@�������̃T���v����
const GLsizei diffuseSamples( 32 );

// �@�������̃~�b�v�}�b�v�̃��x��
const GLint diffuseLod( 5 );

// �T���v���_�̎U�z���a
const GLfloat radius( 0.1f );

// �L���v�`�������摜
GLubyte *buffer;

// �_�Q�̌^���`���Ă���
//typedef pcl::PointXYZ PointType;

// �@���𐄒肷��
/*void estimateNormal( pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals )
{
	// �@������N���X
	pcl::IntegralImageNormalEstimation<PointType, pcl::Normal>  ne;

	ne.setNormalEstimationMethod( ne.AVERAGE_DEPTH_CHANGE );
	ne.setMaxDepthChangeFactor( 0.01 );
	ne.setNormalSmoothingSize( 5.0 );
	ne.setInputCloud( cloud );
	ne.compute( *cloud_normals );
}*/

void main()
{
	// �J�����̎g�p���J�n����
	cv::VideoCapture cap( captureDevice );
	if ( !cap.isOpened() ) return;

	// GLFW ������������
	if ( glfwInit() == GL_FALSE )
	{
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

	// OpenGL�̏����ݒ�
	glClearColor( 0.0f, 0.0f, 0.3f, 0.0f );
	//glEnable( GL_NORMALIZE );
	glEnable( GL_DEPTH_TEST );
	glEnable( GL_CULL_FACE );
	//glEnable( GL_MULTISAMPLE );

	// �A�e�t���𖳌��ɂ���
	//glDisable( GL_LIGHTING );

	// �e�N�X�`��
	GLuint imap[ mapcount ], emap[ mapcount ];
	glGenTextures( mapcount, imap );
	glGenTextures( mapcount, emap );

	// �e�N�X�`���̓ǂݍ���
	for ( size_t i = 0; i < mapcount; ++i )
	{
#if USEMAP
		MyOpenGL::loadMap( irrmaps[ i ], envmaps[ i ], imap[ i ], emap[ i ] );
#else
		MyOpenGL::createMap( skymap, skysize, imap[ 0 ], imapsize, emap[ 0 ], emapsize, ambient, shininess );
#endif
	}

	// ���ˏƓx�}�b�v�̂����グ�Ɏg���e�N�X�`�����j�b�g�̐ݒ�
	glActiveTexture( GL_TEXTURE0 );
	glEnable( GL_TEXTURE_2D );
	MyOpenGL::irradiance();

	// ���ˏƓx�}�b�v�̂����グ�Ɏg���e�N�X�`�����j�b�g�̐ݒ�
	glActiveTexture( GL_TEXTURE1 );
	glEnable( GL_TEXTURE_2D );
	MyOpenGL::diffuse();

	// ���}�b�v�̉��Z�Ɏg���e�N�X�`�����j�b�g�̐ݒ�
	glActiveTexture( GL_TEXTURE2 );
	glEnable( GL_TEXTURE_2D );
	MyOpenGL::reflection();

	// �v���O�����I�u�W�F�N�g���쐬����
	const GLuint program( MyOpenGL::loadProgram( "point.vert", "point.frag" ) );

	// ���s���e�ϊ��s������߂�
	MyOpenGL::cameraMatrix( 30.f, 1.0f, 7.0f, 11.0f, temp1 );

	// ����ϊ��s������߂�
	MyOpenGL::lookAt( 4.0f, 5.0f, 6.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, temp0 );
	// ����ϊ��s��Ɠ��e�ϊ��s��̐ς����߂�
	MyOpenGL::multiplyMatrix( temp0, temp1, projectionMatrix );

	// ���̃e�N�X�`������������
	const auto image( MyOpenGL::createTexture( GL_RGB, cap.get( CV_CAP_PROP_FRAME_WIDTH ), cap.get( CV_CAP_PROP_FRAME_HEIGHT ), diffuseLod ) );

	// uniform�ϐ��̏ꏊ���擾����
	const GLint sizeLoc( glGetUniformLocation( program, "size" ) );
	const GLint scaleLoc( glGetUniformLocation( program, "scale" ) );
	const GLint locationLoc( glGetUniformLocation( program, "location" ) );
	const GLint projectionMatrixLoc( glGetUniformLocation( program, "projectionMatrix" ) );
	const GLint	imapLoc( glGetUniformLocation( program, "imap" ) );
	const GLint imageLoc( glGetUniformLocation( program, "image" ) );
	const GLint diffuseSamplesLoc( glGetUniformLocation( program, "diffuseSamples" ) );
	const GLint duffuseLodLoc( glGetUniformLocation( program, "diffuseLod" ) );
	const GLint radiusLoc( glGetUniformLocation( program, "radius" ) );

	// �}�`�f�[�^���쐬����
	Mesh mesh( filename, false );

	// �B�ʏ���������L���ɂ���
	glEnable( GL_DEPTH_TEST );

	// �E�B���h�E���J���Ă���ԌJ��Ԃ�
	while ( window.shouldClose() == GL_FALSE )
	{
		// �L���v�`�������摜��\������
		cv::Mat frame;
		cap >> frame;
		cv::imshow( "image", frame );

		// �E�B���h�E����������
		glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

		// ���邳
		GLfloat brightness[ 4 ];
		window.getBrightness( brightness );

		// ���ˏƓx�}�b�v�̂����グ
		glActiveTexture( GL_TEXTURE0 );
		glBindTexture( GL_TEXTURE_2D, imap[ 0 ] );
		glTexEnvfv( GL_TEXTURE_ENV, GL_TEXTURE_ENV_COLOR, brightness );

		// �g�U���ˌ����x�̎Z�o
		glActiveTexture( GL_TEXTURE1 );
		glBindTexture( GL_TEXTURE_2D, imap[ 0 ] );

		// ���}�b�s���O
		glActiveTexture( GL_TEXTURE2 );
		glBindTexture( GL_TEXTURE_2D, emap[ 0 ] );

		// �V�F�[�_�v���O�����̎g�p�J�n
		glUseProgram( program );

		// uniform�ϐ��ɒl��ݒ肷��
		glUniform2fv( sizeLoc, 1, window.getSize() );
		glUniform1f( scaleLoc, window.getScale() );
		glUniform2fv( locationLoc, 1, window.getLocation() );
		glUniformMatrix4fv( projectionMatrixLoc, 1, GL_FALSE, projectionMatrix );
		glUniform1i( diffuseSamplesLoc, diffuseSamples );
		glUniform1i( duffuseLodLoc, diffuseLod );
		glUniform1f( radiusLoc, radius );
		glUniform1i( imapLoc, 1 );
		glUniform1i( imageLoc, 2 );
		glActiveTexture( GL_TEXTURE2 );
		glBindTexture( GL_TEXTURE_2D, image );

		// ���̃e�N�X�`���ɉ摜��]������
		glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, cap.get( CV_CAP_PROP_FRAME_WIDTH ), cap.get( CV_CAP_PROP_FRAME_HEIGHT ), GL_RGB, GL_UNSIGNED_BYTE, frame.data );
		glGenerateMipmap( GL_TEXTURE_2D );

		// �}�`��`�悷��
		mesh.draw();
		
		// �J���[�o�b�t�@�����ւ��ăC�x���g�����o��
		window.swapBuffers();

		// ESCAPE�L�[�������ꂽ��I��
		if ( GetKeyState( VK_ESCAPE ) < 0 ) break;
	}

	/*
	try
	{
		// obj�t�@�C����ǂݍ���
		pcl::PolygonMesh::Ptr mesh( new pcl::PolygonMesh() );
		pcl::PointCloud<pcl::PointXYZ>::Ptr obj_pcd( new pcl::PointCloud<pcl::PointXYZ>() );
		if ( pcl::io::loadPolygonFileOBJ( filename, *mesh ) != -1 )
		{
			pcl::fromPCLPointCloud2( mesh->cloud, *obj_pcd );
		}

		// �@��
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals( new pcl::PointCloud < pcl::Normal > );

		// �E�B���h�E�̍쐬
		pcl::visualization::PCLVisualizer viewer( "Point Cloud Viewer" );

		// PointCloud��ǉ�
		viewer.addPointCloud( obj_pcd );

		// �@���𐄒肷��
		estimateNormal( obj_pcd, cloud_normals );
		// �@�����X�V����
		viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>( obj_pcd, cloud_normals, 100, 0.05, "normals" );

		// �E�B���h�E�����Ă��Ȃ��ԑ���
		while ( !viewer.wasStopped() )
		{
			// �X�N���[�����X�V
			viewer.spinOnce();

			// ESCAPE�L�[�������ꂽ��I��
			if ( GetKeyState( VK_ESCAPE ) < 0 )
			{
				break;
			}
		}
	}
	catch ( std::exception& ex )
	{
		std::cout << ex.what() << std::endl;
	}*/
}