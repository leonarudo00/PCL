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
#include <GL/glew.h>
#include <GLFW/glfw3.h>


// objデータを取得
const char filename[] = "bunny.obj";

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
	GLFWwindow *const window( glfwCreateWindow( 640, 480, "Hello!", NULL, NULL ) );
	if ( window == NULL )
	{
		// ウィンドウが作成できなかった
		std::cerr << "Can't create GLFW window." << std::endl;
		glfwTerminate();
		return;
	}

	// 作成したウィンドウを OpenGL の処理対象にする
	glfwMakeContextCurrent( window );

	// GLEW を初期化する
	glewExperimental = GL_TRUE;
	if ( glewInit() != GLEW_OK ){
		// GLEW の初期化に失敗した
		std::cerr << "Can't initialize GLEW" << std::endl;
		return;
	}

	// 垂直同期のタイミングを待つ
	glfwSwapInterval( 1 );

	// 背景色を指定する
	glClearColor( 1.0f, 1.0f, 1.0f, 0.0f );

	// ウィンドウが開いている間繰り返す
	while ( glfwWindowShouldClose( window ) == GL_FALSE ){
		// ウィンドウを消去する
		glClear( GL_COLOR_BUFFER_BIT );

		//
		// ここで描画処理を行う
		//

		// カラーバッファを入れ替える
		glfwSwapBuffers( window );
		// イベントを取り出す
		glfwWaitEvents();
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