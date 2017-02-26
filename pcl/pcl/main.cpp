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

// objデータを取得
const char filename[] = "bunny.obj";



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

	// 背景色を指定する
	glClearColor( 0.0f, 0.0f, 0.3f, 0.0f );

	// プログラムオブジェクトを作成する
	const GLuint program( MyOpenGL::loadProgram( "point.vert", "point.frag" ) );

	// 平行投影変換行列を求める
	//myOpenGL.orthogonalMatrix( -1.0f, 1.0f, -1.0f, 1.0f, 7.0f, 11.0f, temp1 );
	//myOpenGL.perspectiveMatrix( -1.0f, 1.0f, -1.0f, 1.0f, 7.0f, 11.0f, temp1 );
	MyOpenGL::cameraMatrix( 30.f, 1.0f, 7.0f, 11.0f, temp1 );

	// 視野変換行列を求める
	MyOpenGL::lookAt( 4.0f, 5.0f, 6.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, temp0 );
	// 視野変換行列と投影変換行列の積を求める
	MyOpenGL::multiplyMatrix( temp0, temp1, projectionMatrix );

	// uniform変数の場所を取得する
	const GLint sizeLoc( glGetUniformLocation( program, "size" ) );
	const GLint scaleLoc( glGetUniformLocation( program, "scale" ) );
	const GLint locationLoc( glGetUniformLocation( program, "location" ) );
	const GLint projectionMatrixLoc( glGetUniformLocation( program, "projectionMatrix" ) );

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

		// シェーダプログラムの使用開始
		glUseProgram( program );

		// uniform変数に値を設定する
		glUniform2fv( sizeLoc, 1, window.getSize() );
		glUniform1f( scaleLoc, window.getScale() );
		glUniform2fv( locationLoc, 1, window.getLocation() );
		glUniformMatrix4fv( projectionMatrixLoc, 1, GL_FALSE, projectionMatrix );

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