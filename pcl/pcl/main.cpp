// Cのmin,maxマクロを無効にする
#define NOMINMAX
#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <Windows.h>
#include <pcl\visualization\cloud_viewer.h>
#include <pcl\io\vtk_lib_io.h>

// objデータを取得
const char filename[] = "bunny.obj";

void main()
{
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