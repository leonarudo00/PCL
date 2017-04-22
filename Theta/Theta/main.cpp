#include "opencv2\opencv.hpp"

void main()
{
	// �f�o�C�X���I�[�v��
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
		if ( key == 113 )//q�{�^���������ꂽ�Ƃ�
		{
			break;//while���[�v���甲����D
		}
		else if ( key == 115 )//s�������ꂽ�Ƃ�
		{
			//�t���[���摜��ۑ�����D
			cv::imwrite( "img.png", frame );
		}

	}

	cv::destroyAllWindows();
	
	return;
}