#pragma once

//
// OpenCVを使ったキャプチャ
//

// カメラ関連の処理
#include "Camera.h"

// OpenCV
#include <opencv2\highgui\highgui.hpp>

// OpenCVを使ってキャプチャするクラス
class CamCv :public Camera
{
	// OpenCVのキャプチャデバイス
	cv::VideoCapture camera;

	// OpenCVのキャプチャデバイスから取得した画像
	cv::Mat frame;

	// 露出と利得
	int exposure, gain;

	// キャプチャデバイスを初期化する
	bool init( int initial_width, int initial_height, int initial_fps )
	{
		// カメラの解像度を設定する
		if ( initial_width > 0 ) camera.set( CV_CAP_PROP_FRAME_WIDTH, initial_width );
		if ( initial_height > 0 ) camera.set( CV_CAP_PROP_FRAME_HEIGHT, initial_height );
		if ( initial_fps > 0 ) camera.set( CV_CAP_PROP_FPS, initial_fps );

		// カメラから１フレームキャプチャする
		if ( camera.grab() )
		{
			// キャプチャした画像のサイズを取得する
			width = static_cast< GLsizei >( camera.get( CV_CAP_PROP_FRAME_WIDTH ) );
			height = static_cast< GLsizei >( camera.get( CV_CAP_PROP_FRAME_HEIGHT ) );

			// macOSだと設定できても0が返ってくるので対処
			//if ( width == 0 )width = initial_width;
			//if ( height == 0 )height = initial_height;

			// カメラの利得と露出を取得する
			gain = static_cast< GLsizei >( camera.get( CV_CAP_PROP_GAIN ) );
			exposure = static_cast< GLsizei >( camera.get( CV_CAP_PROP_EXPOSURE ) * 10.0 );

			// キャプチャされる画像のフォーマットを設定する
			format = GL_BGR;

			// フレームを取り出してキャプチャ用のメモリを確保する
			camera.retrieve( frame, 3 );

			// 画像がキャプチャされたことを記録する
			buffer = frame.data;

			// カメラが使える
			return true;
		}

		// カメラが使えない
		return false;
	}

	// フレームをキャプチャする
	virtual void capture()
	{
		// あらかじめキャプチャデバイスをロック
		mtx.lock();

		// スレッドが実行可の間
		while ( run )
		{
			// バッファが空のとき次のフレームが到着していれば
			if ( !buffer && camera.grab() )
			{
				// 到着したフレームを切り出して
				camera.retrieve( frame, 3 );

				// 画像を更新
				buffer = frame.data;
			}
			else
			{
				// フレームが切り出せなければロック解除
				mtx.unlock();

				// 他のスレッドがリソースにアクセスするために少し待ってから
				std::this_thread::sleep_for( std::chrono::milliseconds( 10L ) );

				// またキャプチャデバイスをロックする
				mtx.lock();
			}
		}
	}

public:
	// コンストラクタ
	CamCv()
	{
	}

	// デストラクタ
	virtual ~CamCv()
	{
		// スレッドを停止する
		stop();
	}

	// カメラから入力する
	bool open( int device, int width = 0, int height = 0, int fps = 0 )
	{
		// カメラを開く
		camera.open( device );

		// カメラが使えればカメラを初期化する
		if ( camera.isOpened() && init( width, height, fps ) ) return true;

		// カメラが使えない
		return false;
	}

	// ファイル/ネットワークから入力する
	bool open( const std::string &file )
	{
		// ファイル/ネットワークを開く
		camera.open( file );

		// ファイル/ネットワークが使えれば初期化
		if ( camera.isOpened() && init( 0, 0, 0 ) ) return true;

		// ファイル/ネットワークが使えない
		return false;
	}
};