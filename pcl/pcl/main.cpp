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
	// 自作ヘッダ
	MyOpenGL myOpenGL;

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
	const GLuint program( loadProgram( "point.vert", "point.frag" ) );

	// 平行投影変換行列を求める
	//myOpenGL.orthogonalMatrix( -1.0f, 1.0f, -1.0f, 1.0f, 7.0f, 11.0f, temp1 );
	//myOpenGL.perspectiveMatrix( -1.0f, 1.0f, -1.0f, 1.0f, 7.0f, 11.0f, temp1 );
	myOpenGL.cameraMatrix( 30.f, 1.0f, 7.0f, 11.0f, temp1 );

	// 視野変換行列を求める
	myOpenGL.lookAt( 4.0f, 5.0f, 6.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, temp0 );
	// 視野変換行列と投影変換行列の積を求める
	myOpenGL.multiplyMatrix( temp0, temp1, projectionMatrix );

	// uniform変数の場所を取得する
	const GLint sizeLoc( glGetUniformLocation( program, "size" ) );
	const GLint scaleLoc( glGetUniformLocation( program, "scale" ) );
	const GLint locationLoc( glGetUniformLocation( program, "location" ) );
	const GLint projectionMatrixLoc( glGetUniformLocation( program, "projectionMatrix" ) );

	// 図形データを作成する
	std::unique_ptr<const Shape> shape( new Shape( 2, 4, rectangleVertex ) );
	Mesh mesh( 16, 12 );

	// ウィンドウが開いている間繰り返す
	while ( window.shouldClose() == GL_FALSE ){
		// ウィンドウを消去する
		glClear( GL_COLOR_BUFFER_BIT );

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