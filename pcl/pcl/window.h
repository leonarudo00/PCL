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
	GLfloat s;

	// ワールド座標系に対する正規化デバイス座標系の拡大率
	GLfloat scale[ 2 ];

	// 縦横比
	GLfloat aspect;

	// 図形の正規化デバイス座標系上での位置
	GLfloat location[ 2 ];

	// キーボードの状態
	int keyStatus;

	// ワールド座標系に対する正規化デバイス座標系の拡大率を更新する
	void updateScale()
	{
		scale[ 0 ] = s * 2.0f / static_cast<GLfloat>( size[ 0 ] );
		scale[ 1 ] = s * 2.0f / static_cast<GLfloat>( size[ 1 ] );
	}

public:
	// コンストラクタ
	Window( int width = 640, int height = 480, const char *title = "Hello!" )
		: window( glfwCreateWindow( width, height, title, NULL, NULL ) )
		, s( 100.0f )
		, keyStatus( GLFW_RELEASE )
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

		// マウスホイール操作時に呼び出す処理の登録
		glfwSetScrollCallback( window, wheel );

		// キーボード操作時に呼び出す処理の登録
		glfwSetKeyCallback( window, keyboard );

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
		return glfwWindowShouldClose( window ) || glfwGetKey( window, GLFW_KEY_ESCAPE );
	}

	// カラーバッファを入れ替えてイベントを取り出す
	void swapBuffers()
	{
		// カラーバッファを入れ替える
		glfwSwapBuffers( window );

		// イベントを取り出す
		//if ( keyStatus == GLFW_RELEASE ) glfwWaitEvents();
		//else glfwPollEvents();
		glfwPollEvents();

		// キーボードの状態を調べる
		if ( glfwGetKey( window, GLFW_KEY_LEFT ) != GLFW_RELEASE ) location[ 0 ] -= scale[ 0 ] / s;
		else if ( glfwGetKey( window, GLFW_KEY_RIGHT ) != GLFW_RELEASE ) location[ 0 ] += scale[ 0 ] / s;
		if ( glfwGetKey( window, GLFW_KEY_DOWN ) != GLFW_RELEASE ) location[ 1 ] -= scale[ 1 ] / s;
		else if ( glfwGetKey( window, GLFW_KEY_UP ) != GLFW_RELEASE ) location[ 1 ] += scale[ 1 ] / s;

		// マウス左ボタンの状態を調べる
		if ( glfwGetMouseButton( window, GLFW_MOUSE_BUTTON_1 ) != GLFW_RELEASE ){
			// マウスカーソルの位置を取得する
			double x, y;
			glfwGetCursorPos( window, &x, &y );

			// マウスカーソルの正規化デバイス座標系上での位置を求める
			location[ 0 ] = static_cast< GLfloat >( x )* 2.0f / size[ 0 ] - 1.0f;
			location[ 1 ] = 1.0f - static_cast< GLfloat >( y )* 2.0f / size[ 1 ];
		}
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
		return s;
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
			instance->size[ 0 ] = width;
			instance->size[ 1 ] = height;

			// ワールド座標系に対する正規化デバイス座標系の拡大率を更新する
			instance->updateScale();
		}
	}

	// マウスホイール操作時の処理
	static void wheel( GLFWwindow *window, double x, double y )
	{
		// このインスタンスのthisポインタを得る
		Window *const instance( static_cast< Window* >( glfwGetWindowUserPointer( window ) ) );

		if ( instance != NULL ){
			// ワールド座標系に対するデバイス座標系の拡大率を更新する
			instance->s += static_cast< GLfloat >( y );
		}
	}

	// キーボード操作時の処理
	static void keyboard( GLFWwindow *window, int key, int scancode, int action, int mods )
	{
		// このインスタンスのthisポインタを得る
		Window *const instance( static_cast< Window* >( glfwGetWindowUserPointer( window ) ) );

		if ( instance != NULL ){
			// キーの状態を保存する
			instance->keyStatus = action;
		}
	}
};