#pragma once
#define pi	3.14159265f	// 円周率
#include <iostream>
#include <vector>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

namespace MyOpenGL{
	class MyOpenGL
	{
	public:
		// 平行投影変換行列を求める
		void orthogonalMatrix( float left, float right, float bottom, float top, float Near, float Far, GLfloat *matrix )
		{
			float dx = right - left;
			float dy = top - bottom;
			float dz = Far - Near;

			matrix[ 0 ] = 2.0f / dx;
			matrix[ 5 ] = 2.0f / dy;
			matrix[ 10 ] = -2.0f / dz;
			matrix[ 12 ] = -( right + left ) / dx;
			matrix[ 13 ] = -( top + bottom ) / dy;
			matrix[ 14 ] = -( Far + Near ) / dz;
			matrix[ 15 ] = 1.0f;
			matrix[ 1 ] = matrix[ 2 ] = matrix[ 3 ] = matrix[ 4 ] = matrix[ 6 ] = matrix[ 7 ] = matrix[ 8 ] = matrix[ 9 ] = matrix[ 11 ] = 0.0f;

		}

		// 透視投影変換行列を求める
		void perspectiveMatrix( float left, float right, float bottom, float top, float Near, float Far, GLfloat *matrix )
		{
			float dx = right - left;
			float dy = top - bottom;
			float dz = Far - Near;

			matrix[ 0 ] = 2.0f * Near / dx;
			matrix[ 5 ] = 2.0f * Near / dy;
			matrix[ 8 ] = ( right + left ) / dx;
			matrix[ 9 ] = ( top + bottom ) / dy;
			matrix[ 10 ] = -( Far + Near ) / dz;
			matrix[ 11 ] = -1.0f;
			matrix[ 14 ] = -2.0f * Far * Near / dz;
			matrix[ 1 ] = matrix[ 2 ] = matrix[ 3 ] = matrix[ 4 ] = matrix[ 6 ] = matrix[ 7 ] = matrix[ 12 ] = matrix[ 13 ] = matrix[ 15 ] = 0.0f;

		}

		// 視野変換行列を求める
		void lookAt( float ex, float ey, float ez, float tx, float ty, float tz, float ux, float uy, float uz, GLfloat *matrix )
		{
			float l;

			// z軸 = e - t
			tx = ex - tx;
			ty = ey - ty;
			tz = ez - tz;
			l = sqrtf( tx * tx + ty * ty + tz * tz );
			matrix[ 2 ] = tx / l;
			matrix[ 6 ] = ty / l;
			matrix[ 10 ] = tz / l;

			// x軸 = u x z
			tx = uy * matrix[ 10 ] - uz * matrix[ 6 ];
			ty = uz * matrix[ 2 ] - ux * matrix[ 10 ];
			tz = ux * matrix[ 6 ] - uy * matrix[ 2 ];
			l = sqrtf( tx * tx + ty * ty + tz * tz );
			matrix[ 0 ] = tx / l;
			matrix[ 4 ] = ty / l;
			matrix[ 8 ] = tz / l;

			// y軸 = z軸 x x軸
			matrix[ 1 ] = matrix[ 6 ] * matrix[ 8 ] - matrix[ 10 ] * matrix[ 4 ];
			matrix[ 5 ] = matrix[ 10 ] * matrix[ 0 ] - matrix[ 2 ] * matrix[ 8 ];
			matrix[ 9 ] = matrix[ 2 ] * matrix[ 4 ] - matrix[ 6 ] * matrix[ 0 ];

			// 平行移動
			matrix[ 12 ] = -( ex * matrix[ 0 ] + ey * matrix[ 4 ] + ez * matrix[ 8 ] );
			matrix[ 13 ] = -( ex * matrix[ 1 ] + ey * matrix[ 5 ] + ez * matrix[ 9 ] );
			matrix[ 14 ] = -( ex * matrix[ 2 ] + ey * matrix[ 6 ] + ez * matrix[ 10 ] );

			// 残り
			matrix[ 3 ] = matrix[ 7 ] = matrix[ 11 ] = 0.0f;
			matrix[ 15 ] = 1.0f;

		}

		// 行列の積を求める
		void multiplyMatrix( const GLfloat *m0, const GLfloat *m1, GLfloat *matrix )
		{
			for ( int i = 0; i < 16; i++ ){
				int j = i & ~3, k = i & 3;

				matrix[ i ] = m0[ j + 0 ] * m1[ 0 + k ]
					+ m0[ j + 1 ] * m1[ 4 + k ]
					+ m0[ j + 2 ] * m1[ 8 + k ]
					+ m0[ j + 3 ] * m1[ 12 + k ];
			}
		}

		// 画角から透視投影変換行列を求める
		void cameraMatrix( float fovy, float aspect, float Near, float Far, GLfloat *matrix )
		{
			float f = 1.0f / tanf( fovy * 0.5f * 3.141593f / 180.0f );
			float dz = Far - Near;

			matrix[ 0 ] = f / aspect;
			matrix[ 5 ] = f;
			matrix[ 10 ] = -( Far + Near ) / dz;
			matrix[ 11 ] = -1.0f;
			matrix[ 14 ] = -2.0f * Far * Near / dz;
			matrix[ 1 ] = matrix[ 2 ] = matrix[ 3 ] = matrix[ 4 ] =
				matrix[ 6 ] = matrix[ 7 ] = matrix[ 8 ] = matrix[ 9 ] =
				matrix[ 12 ] = matrix[ 13 ] = matrix[ 15 ] = 0.0f;
		}

	};

	// ベクトル
	struct vec3f
	{
		float x, y, z;
	};

	// 面データ
	struct index
	{
		GLuint position[ 3 ];	// 頂点座標番号
		GLuint normal[ 3 ];		// 頂点法線番号
		GLuint texture[ 3 ];	// テクスチャ座標番号
		bool smooth;			// スムーズシェーディングの有無
	};

	// OBJファイルを読み込む
	// name:		OBJファイル名
	// vertexNum:	読み込んだデータの頂点数を格納する変数
	// pos:			頂点の位置のデータを格納したメモリのポインタを格納する変数
	// norm:		頂点の法線データを格納したメモリのポインタを格納する変数
	// faceNum:		読み込んだデータの面数を格納する変数
	// face:		面のデータを格納したメモリのポインタを格納する変数
	// normalize:	trueならサイズを正規化する
	// return:		読み込みに成功したらtrue
	bool loadOBJ( const char *name, GLuint &vertexNum, GLfloat( *&pos )[ 3 ], GLfloat( *&norm )[ 3 ],
		GLuint &faceNum, GLuint( *&face )[ 3 ], bool normalize )
	{
		// OBJファイルの読み込み
		std::ifstream file( name, std::ios::binary );

		// ファイルが開けなかったら戻る
		if ( !file ){
			std::cerr << "Error: Can't open OBJ file: " << name << std::endl;
			return false;
		}

		// 一行読み込み用のバッファ
		std::string line;

		// データの数と座標値の最小値・最大値
		float xmin, xmax, ymin, ymax, zmin, zmax;
		xmax = ymax = zmax = -( xmin = ymin = zmin = FLT_MAX );

		// 頂点位置の一時保存
		std::vector<vec3f> tpos;
		std::vector<index> tface;

		// データを読み込む
		while ( std::getline( file, line ) ){
			std::istringstream str( line );
			std::string op;
			str >> op;

			if ( op == "v" ){
				// 頂点位置
				vec3f v;

				// 頂点位置はスペースで区切られている
				str >> v.x >> v.y >> v.z;

				// 位置の最大値と最小値を求める
				xmin = std::min( xmin, v.x );
				xmax = std::min( xmax, v.x );
				ymin = std::min( ymin, v.y );
				ymax = std::min( ymax, v.y );
				zmin = std::min( zmin, v.z );
				zmax = std::min( zmax, v.z );

				// 頂点データを保存する
				tpos.push_back( v );
			}
			else if ( op == "f" ){
				// 面データ
				index f;
			}
		}
	}
}