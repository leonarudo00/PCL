#pragma once
#include <iostream>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

// ウィンドウ関連の処理
class Window
{
	// ウィンドウのハンドル
	GLFWwindow *const window;

	// ウィンドウのサイズ
	GLfloat size[ 2 ];

	// ワールド座標系に対するデバイス座標系の拡大率
	GLfloat scale;

	// 縦横比
	GLfloat aspect;

	// 図形の正規化デバイス座標系上での位置
	GLfloat location[ 2 ];

public:
	// コンストラクタ
	Window( int width = 640, int height = 480, const char *title = "Hello!" )
		: window( glfwCreateWindow( width, height, title, NULL, NULL ) )
		, scale( 100.0f )
	{
		if ( window == NULL ){
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

		// このインスタンスのthisポインタを記録しておく
		glfwSetWindowUserPointer( window, this );

		// ウィンドウのサイズ変更時に呼び出す処理の登録
		glfwSetWindowSizeCallback( window, resize );

		// 開いたウィンドウの初期設定
		resize( window, width, height );

		// 図形の正規化デバイス座標系上での位置の初期値
		location[ 0 ] = location[ 1 ] = 0.0f;
	}

	// デストラクタ
	virtual ~Window()
	{
		glfwDestroyWindow( window );
	}

	// ウィンドウを閉じるべきかを判定する
	int shouldClose() const
	{
		return glfwWindowShouldClose( window );
	}

	// カラーバッファを入れ替えてイベントを取り出す
	void swapBuffers()
	{
		// カラーバッファを入れ替える
		glfwSwapBuffers( window );

		// イベントを取り出す
		glfwWaitEvents();

		// マウスカーソルの位置を取得する
		double x, y;
		glfwGetCursorPos( window, &x, &y );

		// マウスカーソルの正規化デバイス座標系上での位置を求める
		location[ 0 ] = static_cast< GLfloat >( x )* 2.0f / size[ 0 ] - 1.0f;
		location[ 1 ] = 1.0f - static_cast< GLfloat >( y )* 2.0f / size[ 1 ];
	}

	// 縦横比を取り出す
	GLfloat getAspect() const
	{
		return aspect;
	}

	// ウィンドウのサイズを取り出す
	const GLfloat *getSize() const
	{
		return size;
	}

	// ワールド座標系に対するデバイス座標系の拡大率を取り出す
	GLfloat getScale() const
	{
		return scale;
	}

	// 位置を取り出す
	const GLfloat *getLocation() const
	{
		return location;
	}

	// ウィンドウのサイズ変更時の処理
	static void resize( GLFWwindow *const window, int width, int height )
	{
		// ウィンドウ全体をビューポートに設定する
		glViewport( 0, 0, width, height );

		// このインスタンスのthisポインタを得る
		Window *const instance( static_cast< Window* >( glfwGetWindowUserPointer( window ) ) );

		if ( instance != NULL ){
			// 開いたウィンドウのサイズを保存する
			instance->size[ 0 ] = static_cast< GLfloat >( width );
			instance->size[ 1 ] = static_cast< GLfloat >( height );
		}
	}
};