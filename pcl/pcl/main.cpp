// Cのmin,maxマクロを無効にする
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

// 事前計算したマップを使用するなら 1
#define USEMAP 1

// objデータを取得
const char filename[] = "bunny.obj";

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

// 放射照度マップの数
const size_t mapcount( sizeof irrmaps / sizeof irrmaps[ 0 ] );

// 天空画像
const char skymap[] = "skymap0.tga";

// 大域環境光強度
const GLfloat ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };

// 輝き係数
const GLfloat shininess( 60.0f );

// 天空画像中の天空領域の直径の最大値
const GLsizei skysize( 1024 );

// 作成するテクスチャのサイズ
const GLsizei imapsize( 256 );
const GLsizei emapsize( 256 );

// 矩形の頂点の位置
const Object::Vertex rectangleVertex[]=
{
	{ -0.5f, -0.5f },
	{  0.5f, -0.5f },
	{  0.5f,  0.5f },
	{ -0.5f,  0.5 }
};

GLfloat projectionMatrix[ 16 ];		// 投影変換行列
GLfloat temp0[ 16 ], temp1[ 16 ];	// 一時的な変換行列

void main()
{
	// GLFW を初期化する
	if ( glfwInit() == GL_FALSE ){
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
	glClearColor( 0.0f, 0.0f, 0.3f, 0.0f );
	//glEnable( GL_NORMALIZE );
	glEnable( GL_DEPTH_TEST );
	glEnable( GL_CULL_FACE );
	//glEnable( GL_MULTISAMPLE );

	// 陰影付けを無効にする
	//glDisable( GL_LIGHTING );

	// テクスチャ
	GLuint imap[ mapcount ], emap[ mapcount ];
	glGenTextures( mapcount, imap );
	glGenTextures( mapcount, emap );

	// テクスチャの読み込み
	for ( size_t i = 0; i < mapcount; ++i ){
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

	// プログラムオブジェクトを作成する
	const GLuint program( MyOpenGL::loadProgram( "point.vert", "point.frag" ) );

	// 平行投影変換行列を求める
	MyOpenGL::cameraMatrix( 30.f, 1.0f, 7.0f, 11.0f, temp1 );

	// 視野変換行列を求める
	MyOpenGL::lookAt( 4.0f, 5.0f, -6.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, temp0 );
	// 視野変換行列と投影変換行列の積を求める
	MyOpenGL::multiplyMatrix( temp0, temp1, projectionMatrix );

	// uniform変数の場所を取得する
	const GLint sizeLoc( glGetUniformLocation( program, "size" ) );
	const GLint scaleLoc( glGetUniformLocation( program, "scale" ) );
	const GLint locationLoc( glGetUniformLocation( program, "location" ) );
	const GLint projectionMatrixLoc( glGetUniformLocation( program, "projectionMatrix" ) );
	const GLint	imapLoc( glGetUniformLocation( program, "imap" ) );

	// 図形データを作成する
	std::unique_ptr<const Shape> shape( new Shape( 2, 4, rectangleVertex ) );
	//Mesh mesh( 16, 12 );
	Mesh mesh( filename, false );

	// 隠面消去処理を有効にする
	glEnable( GL_DEPTH_TEST );

	// ウィンドウが開いている間繰り返す
	while ( window.shouldClose() == GL_FALSE ){
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
		glUniform2fv( sizeLoc, 1, window.getSize() );
		glUniform1f( scaleLoc, window.getScale() );
		glUniform2fv( locationLoc, 1, window.getLocation() );
		glUniformMatrix4fv( projectionMatrixLoc, 1, GL_FALSE, projectionMatrix );
		glUniform1i( imapLoc, 2 );

		// 図形を描画する
		//shape->draw();
		mesh.draw();

		// カラーバッファを入れ替えてイベントを取り出す
		window.swapBuffers();
	}

	try{
		// objファイルを読み込む
		pcl::PolygonMesh::Ptr mesh( new pcl::PolygonMesh() );
		pcl::PointCloud<pcl::PointXYZ>::Ptr obj_pcd( new pcl::PointCloud<pcl::PointXYZ>() );
		if ( pcl::io::loadPolygonFileOBJ( filename, *mesh ) != -1 ){
			pcl::fromPCLPointCloud2( mesh->cloud, *obj_pcd );
		}

		// ウィンドウの作成
		pcl::visualization::PCLVisualizer viewer( "Point Cloud Viewer" );

		// PointCloudを追加
		viewer.addPointCloud( obj_pcd );

		// ウィンドウが閉じていない間続く
		while ( !viewer.wasStopped() ) {
			// スクリーンを更新
			viewer.spinOnce();

			// ESCAPEキーが押されたら終了
			if ( GetKeyState( VK_ESCAPE ) < 0 ){
				break;
			}
		}
	}
	catch ( std::exception& ex ){
		std::cout << ex.what() << std::endl;
	}
}