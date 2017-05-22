// Cのmin,maxマクロを無効にする
#define NOMINMAX
#define _CRT_SECURE_NO_WARNINGS

// 事前計算した放射照度マップを使用するなら 1
#define USEMAP 1

#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "glfw3.lib")
#pragma comment(lib, "glew32.lib")

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
#include <Windows.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "opencv2\opencv.hpp"
//#include <pcl\visualization\cloud_viewer.h>
//#include <pcl\point_types.h> 
//#include <pcl\io\vtk_lib_io.h>#include "window.h"
#include "window.h"
#include "Mesh.h"
#include "MyOpenGL.h"

// 投影変換行列
GLfloat projectionMatrix[ 16 ];

// 一時的な変換行列
GLfloat temp0[ 16 ], temp1[ 16 ];

// キャプチャに用いるカメラのデバイス番号
const int captureDevice( 1 );

// キャプチャするフレームのサイズ (0 ならデフォルト)
const int captureWidth( 1280 ), captureHeight( 720 );

// objデータ名
const char *const filename[] = 
{
	"ball.obj",			// 0
	"bunny.obj",		// 1
	"mario.obj",		// 2
	"colorChecker.obj"	// 3
};
// 使用するobjファイル番号
const int objFile( 3 );

//
// 放射照度マップによる陰影付けで使う変数群
//
// 放射照度マップ
const char *const irrmaps[]=
{
	"irr0.tga"
};

// 環境マップ
const char *const envmaps[]=
{
	"env0.tga"
};

// 天空画像
const char skymap[] = "skymap0.tga";

// 放射照度マップの数
const size_t mapcount( sizeof irrmaps / sizeof irrmaps[ 0 ] );

// 大域環境光強度
const GLfloat ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };

// 輝き係数
const GLfloat shininess( 60.0f );

// 天空画像中の天空領域の直径の最大値
const GLsizei skysize( 1024 );

// 作成するテクスチャのサイズ
const GLsizei imapsize( 256 );
const GLsizei emapsize( 256 );

//
// DualFisheye画像のサンプリングによる陰影付けで使う変数群
//
// 拡散反射光のサンプル数
const GLsizei diffuseSamples( 200 );

// 拡散反射光をサンプルする際のミップマップのレベル
const GLint diffuseLod( 5 );

// 鏡面反射光のサンプル数
const GLsizei specularSamples( 1 );

// 鏡面反射光をサンプルする際のミップマップのレベル
const GLint specularLod( 0 );

// サンプル点の散布半径
const GLfloat radius( 0.1f );

// カラーチェッカー画像
const char colorCheckerFile[] = "colorChecker.tga";

// 点群の型を定義しておく
//typedef pcl::PointXYZ PointType;

// 法線を推定する
/*void estimateNormal( pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals )
{
	// 法線推定クラス
	pcl::IntegralImageNormalEstimation<PointType, pcl::Normal>  ne;

	ne.setNormalEstimationMethod( ne.AVERAGE_DEPTH_CHANGE );
	ne.setMaxDepthChangeFactor( 0.01 );
	ne.setNormalSmoothingSize( 5.0 );
	ne.setInputCloud( cloud );
	ne.compute( *cloud_normals );
}*/


void main()
{
	// カメラの使用を開始する
	cv::VideoCapture cap( captureDevice );
	if ( !cap.isOpened() )
	{
		std::cerr << "Can't open camera." << std::endl;
		return;
	}

	// カメラ解像度を設定する
	cap.set( CV_CAP_PROP_FRAME_WIDTH, captureWidth );
	cap.set( CV_CAP_PROP_FRAME_HEIGHT, captureHeight );

	// GLFW を初期化する
	if ( glfwInit() == GL_FALSE )
	{
		// 初期化に失敗した
		std::cerr << "Can't initialize GLFW" << std::endl;
		return;
	}

	// プログラム終了時の処理を登録する
	atexit( glfwTerminate );

	// OpenGL Version 3.2 Core Profile を選択する
	glfwWindowHint( GLFW_CONTEXT_VERSION_MAJOR, 3 );
	glfwWindowHint( GLFW_CONTEXT_VERSION_MINOR, 2 );
	glfwWindowHint( GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE );
	glfwWindowHint( GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE );

	// ウィンドウを作成する
	Window window;

	// OpenGLの初期設定
	glClearColor( 1.0f, 1.0f, 1.0f, 0.0f );
	//glEnable( GL_NORMALIZE );
	glEnable( GL_DEPTH_TEST );
	glEnable( GL_CULL_FACE );
	//glEnable( GL_MULTISAMPLE );
	//glDisable( GL_LIGHTING );

	// 放射照度マップと環境マップの準備
	GLuint imap[ mapcount ], emap[ mapcount ];
	glGenTextures( mapcount, imap );
	glGenTextures( mapcount, emap );

	// テクスチャに各マップを読み込む
	for ( size_t i = 0; i < mapcount; ++i )
	{
#if USEMAP
		MyOpenGL::loadMap( irrmaps[ i ], envmaps[ i ], imap[ i ], emap[ i ] );
#else
		MyOpenGL::createMap( skymap, skysize, imap[ 0 ], imapsize, emap[ 0 ], emapsize, ambient, shininess );
#endif
	}

	// 放射照度マップのかさ上げに使うテクスチャユニットの設定
	glActiveTexture( GL_TEXTURE0 );
	glEnable( GL_TEXTURE_2D );
	MyOpenGL::irradiance();

	// 放射照度マップのかさ上げに使うテクスチャユニットの設定
	glActiveTexture( GL_TEXTURE1 );
	glEnable( GL_TEXTURE_2D );
	MyOpenGL::diffuse();

	// 環境マップの加算に使うテクスチャユニットの設定
	glActiveTexture( GL_TEXTURE2 );
	glEnable( GL_TEXTURE_2D );
	MyOpenGL::reflection();

	// 投影変換行列を求める
	MyOpenGL::cameraMatrix( 30.f, 1.0f, 5.0f, 20.0f, temp1 );
	// 視野変換行列を求める
	MyOpenGL::lookAt( 4.0f, 5.0f, 6.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, temp0 );
	// 透視投影変換行列を求める
	MyOpenGL::multiplyMatrix( temp0, temp1, projectionMatrix );
	// 法線ベクトルを視野座標系に変換する行列を求める
	GLfloat transNormalMat[ 16 ];
	MyOpenGL::loadNormal( temp0, transNormalMat );

	// 環境のテクスチャを準備する
	const auto envImage( MyOpenGL::createTexture( GL_RGB, cap.get( CV_CAP_PROP_FRAME_WIDTH ), cap.get( CV_CAP_PROP_FRAME_HEIGHT ), diffuseLod ) );

	// プログラムオブジェクトを作成する
	const GLuint program( MyOpenGL::loadProgram( "point.vert", "point.frag" ) );

	// uniform変数の場所を取得する
	// point.vert/.frag
	const GLint colorLoc			( glGetUniformLocation( program, "color" ) );
	const GLint colorCheckerImageLoc( glGetUniformLocation( program, "colorCheckerImage" ) );
	const GLint diffuseSamplesLoc	( glGetUniformLocation( program, "diffuseSamples" ) );
	const GLint diffuseLodLoc		( glGetUniformLocation( program, "diffuseLod" ) );
	const GLint envImageLoc			( glGetUniformLocation( program, "envImage" ) );
	const GLint	irrmapLoc			( glGetUniformLocation( program, "irrmap" ) );
	const GLint locationLoc			( glGetUniformLocation( program, "location" ) );
	const GLint projectionMatrixLoc	( glGetUniformLocation( program, "projectionMatrix" ) );
	const GLint radiusLoc			( glGetUniformLocation( program, "radius" ) );
	const GLint scaleLoc			( glGetUniformLocation( program, "scale" ) );
	const GLint sizeLoc				( glGetUniformLocation( program, "size" ) );
	const GLint specularLodLoc		( glGetUniformLocation( program, "specularLod" ) );
	const GLint specularSamplesLoc	( glGetUniformLocation( program, "specularSamples" ) );
	const GLint transNormalMatLoc	( glGetUniformLocation( program, "transNormalMat" ) );
	const GLint transViewMat		( glGetUniformLocation( program, "transViewMat" ) );

	// 図形データを作成する
	Mesh mesh( filename[objFile], false );
	//Mesh mesh( 10, 10 );
	//Mesh mesh;

	// 隠面消去処理を有効にする
	glEnable( GL_DEPTH_TEST );

	GLuint colorCheckerTexture;
	GLsizei colorCheckerWidth, colorCheckerHeight;
	GLenum colorCheckerFormat;
	glGenTextures( 1, &colorCheckerTexture );
	GLubyte *colorCheckerBuffer = MyOpenGL::loadTga( colorCheckerFile, &colorCheckerWidth, &colorCheckerHeight, &colorCheckerFormat );
	glActiveTexture( GL_TEXTURE4 );
	glBindTexture( GL_TEXTURE_2D, colorCheckerTexture );
	glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, colorCheckerWidth, colorCheckerHeight, 0, colorCheckerFormat, GL_UNSIGNED_BYTE, colorCheckerBuffer );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );

	
	// ウィンドウが開いている間繰り返す
	while ( window.shouldClose() == GL_FALSE )
	{
		// キャプチャした画像を表示する
		cv::Mat frame;
		cap >> frame;
		if ( !frame.empty() ) cv::imshow( "image", frame );

		// ウィンドウを消去する
		glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

		// 明るさ
		GLfloat brightness[ 4 ];
		window.getBrightness( brightness );

		// 放射照度マップのかさ上げ
		glActiveTexture( GL_TEXTURE0 );
		glBindTexture( GL_TEXTURE_2D, imap[ 0 ] );
		glTexEnvfv( GL_TEXTURE_ENV, GL_TEXTURE_ENV_COLOR, brightness );

		// 拡散反射光強度の算出
		glActiveTexture( GL_TEXTURE1 );
		glBindTexture( GL_TEXTURE_2D, imap[ 0 ] );

		// 環境マッピング
		glActiveTexture( GL_TEXTURE2 );
		glBindTexture( GL_TEXTURE_2D, emap[ 0 ] );

		// シェーダプログラムの使用開始
		glUseProgram( program );

		// uniform変数に値を設定する
		glUniform1i			( colorCheckerImageLoc, 4 );
		glUniform1i			( diffuseLodLoc,		diffuseLod );
		glUniform1i			( diffuseSamplesLoc,	diffuseSamples );
		glUniform1i			( envImageLoc,			5 );
		glUniform1i			( irrmapLoc,			1 );
		glUniform1i			( specularLodLoc,		specularLod );
		glUniform1i			( specularSamplesLoc,	specularSamples );
		glUniform1f			( radiusLoc,			radius );
		glUniform1f			( scaleLoc,				window.getScale() );
		glUniform2fv		( locationLoc,			1, window.getLocation() );
		glUniform2fv		( sizeLoc,				1, window.getSize() );
		glUniformMatrix4fv	( projectionMatrixLoc,  1, GL_FALSE, projectionMatrix );
		glUniformMatrix4fv	( transNormalMatLoc,	1, GL_FALSE, transNormalMat );
		glUniformMatrix4fv	( transViewMat,			1, GL_FALSE, temp0 );

		// テクスチャ番号２に環境マップを割り当てる
		glActiveTexture( GL_TEXTURE5 );
		glBindTexture( GL_TEXTURE_2D, envImage );

		// 環境のテクスチャに画像を転送する
		glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, cap.get( CV_CAP_PROP_FRAME_WIDTH ), cap.get( CV_CAP_PROP_FRAME_HEIGHT ), GL_RGB, GL_UNSIGNED_BYTE, frame.data );
		glGenerateMipmap( GL_TEXTURE_2D );

		// 図形を描画する
		mesh.draw();

		// カラーバッファを入れ替えてイベントを取り出す
		window.swapBuffers();

		// ESCAPEキーが押されたら終了
		if ( GetKeyState( VK_ESCAPE ) < 0 ) break;
	}

	/*
	try
	{
		// objファイルを読み込む
		pcl::PolygonMesh::Ptr mesh( new pcl::PolygonMesh() );
		pcl::PointCloud<pcl::PointXYZ>::Ptr obj_pcd( new pcl::PointCloud<pcl::PointXYZ>() );
		if ( pcl::io::loadPolygonFileOBJ( filename, *mesh ) != -1 )
		{
			pcl::fromPCLPointCloud2( mesh->cloud, *obj_pcd );
		}

		// 法線
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals( new pcl::PointCloud < pcl::Normal > );

		// ウィンドウの作成
		pcl::visualization::PCLVisualizer viewer( "Point Cloud Viewer" );

		// PointCloudを追加
		viewer.addPointCloud( obj_pcd );

		// 法線を推定する
		estimateNormal( obj_pcd, cloud_normals );
		// 法線を更新する
		viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>( obj_pcd, cloud_normals, 100, 0.05, "normals" );

		// ウィンドウが閉じていない間続く
		while ( !viewer.wasStopped() )
		{
			// スクリーンを更新
			viewer.spinOnce();

			// ESCAPEキーが押されたら終了
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