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
#include <fstream>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

// objデータを取得
const char filename[] = "bunny.obj";

// シェーダオブジェクトのコンパイル結果を表示する
// shader: シェーダオブジェクト名
// str: コンパイルエラーが発生した場所を示す文字列
GLboolean printShaderInfoLog( GLuint shader, const char *str )
{
	// コンパイル結果を取得する
	GLint status;
	glGetShaderiv( shader, GL_COMPILE_STATUS, &status );
	if ( status == GL_FALSE ) std::cerr << "Compile Error in " << str << std::endl;
	
	// シェーダのコンパイル時のログの長さを取得する
	GLsizei bufSize;
	glGetShaderiv( shader, GL_INFO_LOG_LENGTH, &bufSize );
	if ( bufSize > 1 ){
		// シェーダのコンパイル時のログの内容を取得する
		std::vector<GLchar> infoLog( bufSize );
		GLsizei length;
		glGetShaderInfoLog( shader, bufSize, &length, &infoLog[ 0 ] );
		std::cerr << &infoLog[ 0 ] << std::endl;
	}

	return static_cast<GLboolean>( status );
}

// プログラムオブジェクトのリンク結果を表示する
// program: プログラムオブジェクト名
GLboolean printProgramInfoLog( GLuint program )
{
	// リンク結果を取得する
	GLint status;
	glGetProgramiv( program, GL_LINK_STATUS, &status );
	if ( status == GL_FALSE ) std::cerr << "Link Error." << std::endl;
	
	// シェーダのリンク時のログの長さを取得する
	GLsizei bufSize;
	glGetProgramiv( program, GL_INFO_LOG_LENGTH, &bufSize );
	
	if ( bufSize > 1 ){
		// シェーダのリンク時のログの内容を取得する
		std::vector<GLchar> infoLog( bufSize );
		GLsizei length;
		glGetProgramInfoLog( program, bufSize, &length, &infoLog[ 0 ] );
		std::cerr << &infoLog[ 0 ] << std::endl;
	}

	return static_cast<GLboolean>( status );
}

// プログラムオブジェクトを作成する
// vsrc: バーテックスシェーダのソースプログラムの文字列
// fsrc: フラグメントシェーダのソースプログラムの文字列
GLuint createProgram( const char *vsrc, const char *fsrc )
{
	// 空のプログラムオブジェクトを作成する
	const GLuint program( glCreateProgram() );

	if ( vsrc != NULL ){
		// バーテックスシェーダのシェーダオブジェクトを作成する
		const GLuint vobj( glCreateShader( GL_VERTEX_SHADER ) );
		glShaderSource( vobj, 1, &vsrc, NULL );
		glCompileShader( vobj );
		
		// バーテックスシェーダのシェーダオブジェクトをプログラムオブジェクトに組み込む
		if ( printShaderInfoLog( vobj, "vertex shader" ) ) glAttachShader( program, vobj );
		glDeleteShader( vobj );
	}

	if ( fsrc != NULL ){
		// フラグメントシェーダのシェーダオブジェクトを作成する
		const GLuint fobj( glCreateShader( GL_FRAGMENT_SHADER ) );
		glShaderSource( fobj, 1, &fsrc, NULL );
		glCompileShader( fobj );
	
		// フラグメントシェーダのシェーダオブジェクトをプログラムオブジェクトに組み込む
		if ( printShaderInfoLog( fobj, "fragment shader" ) ) glAttachShader( program, fobj );
		glDeleteShader( fobj );
	}

	// プログラムオブジェクトをリンクする
	glBindAttribLocation( program, 0, "position" );
	glBindFragDataLocation( program, 0, "fragment" );
	glLinkProgram( program );
	
	// 作成したプログラムオブジェクトを返す
	if ( printProgramInfoLog( program ) ) return program;

	// プログラムオブジェクトが作成できなければ 0 を返す
	glDeleteProgram( program );
	return 0;
}
// シェーダのソースファイルを読み込んだメモリを返す
// name: シェーダのソースファイル名
// buffer: 読み込んだソースファイルのテキスト
bool readShaderSource( const char *name, std::vector<GLchar> &buffer )
{
	// ファイル名がNULLだった
	if ( name == NULL )return false;

	// ソースファイルを開く
	std::ifstream file( name, std::ios::binary );
	if ( file.fail() ){
		// 開けなかった
		std::cerr << "Error: Can't open source file: " << name << std::endl;
		return false;
	}

	// ファイルの末尾に移動し現在位置（＝ファイルサイズ）を得る
	file.seekg( 0L, std::ios::end );
	GLsizei length = static_cast< GLsizei >( file.tellg() );

	// ファイルサイズのメモリを確保
	buffer.resize( length + 1 );

	// ファイルを先頭から読み込む
	file.seekg( 0L, std::ios::beg );
	file.read( buffer.data(), length );
	buffer[ length ] = '\0';

	if ( file.fail() ){
		// うまく読み込めなかった
		std::cerr << "Error: Could not read source file: " << name << std::endl;
		file.close();
		return false;
	}

	// 読み込み成功
	file.close();
	return true;
}

// シェーダのソースファイルを読み込んでプログラムオブジェクトを作成する
// vert: バーテックスシェーダのソースファイル名
// frag: フラグメントシェーダのソースファイル名
GLuint loadProgram( const char *vert, const char *frag )
{
	// シェーダのソースファイルを読み込む
	std::vector<GLchar> vsrc;
	const bool vstat( readShaderSource( vert, vsrc ) );
	std::vector<GLchar> fsrc;
	const bool fstat( readShaderSource( frag, fsrc ) );
	// プログラムオブジェクトを作成する
	return vstat && fstat ? createProgram( vsrc.data(), fsrc.data() ) : 0;
}

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

	// プログラムオブジェクトを作成する
	const GLuint program( loadProgram( "point.vert", "point.frag" ) );

	// ウィンドウが開いている間繰り返す
	while ( glfwWindowShouldClose( window ) == GL_FALSE ){
		// ウィンドウを消去する
		glClear( GL_COLOR_BUFFER_BIT );

		// シェーダプログラムの使用開始
		glUseProgram( program );

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