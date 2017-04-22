#include "opencv2\opencv.hpp"

void main()
{
	// デバイスをオープン
	cv::VideoCapture cap( 1 );

	if ( cap.isOpened() == false )
	{
		return;
	}

	//cap.set( CV_CAP_PROP_FRAME_WIDTH, 2048 );
	//cap.set( CV_CAP_PROP_FRAME_HEIGHT, 1024 );

	while ( 1 )
	{
		cv::Mat frame;
		cap >> frame;

		cv::imshow( "image", frame );

		int key = cv::waitKey( 1 );
		if ( key == 113 )//qボタンが押されたとき
		{
			break;//whileループから抜ける．
		}
		else if ( key == 115 )//sが押されたとき
		{
			//フレーム画像を保存する．
			cv::imwrite( "img.png", frame );
		}

	}

	cv::destroyAllWindows();
	
	return;
}