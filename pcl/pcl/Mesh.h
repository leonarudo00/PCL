#pragma once

//
// メッシュ
//

// 自作ヘッダ
#include "MyOpenGL.h"

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/integral_image_normal.h>

class Mesh
{
	// 頂点バッファオブジェクトのメモリを参照するポインタ
	typedef GLfloat Position[ 3 ];
	typedef GLfloat Normal[ 3 ];
	typedef GLuint Face[ 3 ];
	Position *position;
	Normal *normal;
	Face *face;

	GLuint	vertices;			// 頂点数

	GLuint	indexes;			// インデックスの数

	GLuint	vao;				// 頂点配列オブジェクト

	GLuint	vao2;
	
	GLuint	positionBuffer;		// 頂点位置を格納する頂点バッファオブジェクト

	GLuint	indexBuffer;		// 頂点インデックスを格納する頂点バッファオブジェクト

	GLuint	normalBuffer;		// 頂点法線を格納する頂点バッファオブジェクト

	GLuint	normalPointsBuffer;	// 法線を描画するための頂点位置を格納する頂点バッファオブジェクト

	int		frame = 0;			// フレーム

	int		cycle = 100;		// 周波数

	int		slices;				// 横の頂点数

	int		stacks;				// 縦の頂点数

public:
	// コンストラクタ
	// slices: 横の頂点数
	// stacks: 縦の頂点数
	Mesh( int slices, int stacks )
		:vertices( slices * stacks )
		, indexes( ( slices - 1 )*( stacks - 1 ) * 2 * 3 )
		, slices( slices )
		, stacks( stacks )
	{

		// 頂点配列オブジェクトを作成する
		glGenVertexArrays( 1, &vao );
		glBindVertexArray( vao );

		// 頂点バッファオブジェクトを作成する
		glGenBuffers( 1, &positionBuffer );
		glBindBuffer( GL_ARRAY_BUFFER, positionBuffer );
		glGenBuffers( 1, &indexBuffer );
		glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, indexBuffer );

		// 頂点バッファオブジェクトにメモリ領域を確保する
		glBufferData( GL_ARRAY_BUFFER, sizeof( Position ) * vertices, NULL, GL_DYNAMIC_DRAW );
		glBufferData( GL_ELEMENT_ARRAY_BUFFER, sizeof( Face ) * indexes, NULL, GL_STATIC_DRAW );

		// 頂点バッファオブジェクトのメモリをプログラムのメモリ空間にマップする
		position = ( Position* )glMapBuffer( GL_ARRAY_BUFFER, GL_WRITE_ONLY );
		face = ( Face* )glMapBuffer( GL_ELEMENT_ARRAY_BUFFER, GL_WRITE_ONLY );

		// 頂点位置を設定する
		for ( int j = 0; j < stacks; ++j ){
			for ( int i = 0; i < slices; ++i ){
				const auto x( ( GLfloat( i ) / GLfloat( slices - 1 ) - 0.5f ) * GLfloat( slices ) / GLfloat( stacks ) );
				const auto y( ( GLfloat( j ) / GLfloat( stacks - 1 ) - 0.5f ) );

				( *position )[ 0 ] = x;
				( *position )[ 1 ] = y;
				( *position )[ 2 ] = 0.0f;
				++position;
			}
		}

		// 頂点バッファオブジェクトのメモリをプログラムのメモリ空間から切り離す
		glUnmapBuffer( GL_ARRAY_BUFFER );

		// 頂点バッファオブジェクトをattribute変数に対応づける
		glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, 0, 0 );
		// 0番のattrib変数を使用可能にする
		glEnableVertexAttribArray( 0 );

		// インデックスを設定する
		for ( int j = 0; j < stacks - 1; ++j ){
			for ( int i = 0; i < slices - i; ++i ){
				int k = slices * j + i;
				
				( *face )[ 0 ] = k;
				( *face )[ 1 ] = k + 1;
				( *face )[ 2 ] = k + slices;
				++face;

				( *face )[ 0 ] = k + slices + 1;
				( *face )[ 1 ] = k + slices;
				( *face )[ 2 ] = k + 1;
				++face;
			}
		}

		// 頂点バッファオブジェクトのメモリをプログラムのメモリ空間から切り離す
		glUnmapBuffer( GL_ELEMENT_ARRAY_BUFFER );

		// 頂点バッファオブジェクトの結合を解除する
		glBindVertexArray( 0 );

	}
	// コンストラクタ
	// name: ファイル名
	// normalize: 正規化の有無
	Mesh( const char *name, bool normalize )
	{
		// OBJファイルを読み込む
		MyOpenGL::loadOBJ( name, vertices, position, normal, indexes, face, normalize );

		// PCLでOBJファイルを読み込む
		pcl::PolygonMesh::Ptr mesh( new pcl::PolygonMesh() );
		pcl::PointCloud < pcl::PointXYZ >::Ptr obj_pcd( new pcl::PointCloud<pcl::PointXYZ>() );
		if ( pcl::io::loadPolygonFileOBJ( name, *mesh ) != -1 ){
			pcl::fromPCLPointCloud2( mesh->cloud, *obj_pcd );
		}

		// 頂点配列オブジェクトを作成する
		glGenVertexArrays( 1, &vao );
		glBindVertexArray( vao );

		// 頂点バッファオブジェクトを作成する
		glGenBuffers( 1, &positionBuffer );
		glBindBuffer( GL_ARRAY_BUFFER, positionBuffer );
		glGenBuffers( 1, &normalBuffer );
		glBindBuffer( GL_ARRAY_BUFFER, normalBuffer );
		glGenBuffers( 1, &indexBuffer );
		glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, indexBuffer );

		// 頂点位置バッファオブジェクトにメモリ領域を確保する
		glBufferData( GL_ARRAY_BUFFER, sizeof( Position ) * vertices, NULL, GL_DYNAMIC_DRAW );
		// 頂点バッファオブジェクトのメモリをプログラムのメモリ空間にマップする
		position = ( Position* )glMapBuffer( GL_ARRAY_BUFFER, GL_WRITE_ONLY );
		// 法線を描画する頂点の位置を設定する
		for ( int i = 0; i < vertices; ++i ){
			( *position )[ 0 ] = obj_pcd->points[ i ].x;
			( *position )[ 1 ] = obj_pcd->points[ i ].y;
			( *position )[ 2 ] = obj_pcd->points[ i ].z;
			++position;
		}
		// attribute変数に対応づける
		glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, 0, 0 );
		// 0番のattrib変数を使用可能にする
		glEnableVertexAttribArray( 0 );
		// メモリ空間から切り離す
		glUnmapBuffer( GL_ARRAY_BUFFER );
		glBindBuffer( GL_ARRAY_BUFFER, 0 );

		// 頂点法線位置バッファオブジェクトにメモリ領域を確保する
		glBufferData( GL_ARRAY_BUFFER, sizeof( Normal ) * vertices, normal, GL_DYNAMIC_DRAW );
		// attribute変数に対応づける
		glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, 0, 0 );
		// 1番のattrib変数を使用可能にする
		glEnableVertexAttribArray( 1 );
		// メモリ空間から切り離す
		glUnmapBuffer( GL_ARRAY_BUFFER );
		glBindBuffer( GL_ARRAY_BUFFER, 0 );

		// 頂点インデックスバッファオブジェクトにメモリ領域を確保する
		glBufferData( GL_ELEMENT_ARRAY_BUFFER, sizeof( Face ) * indexes, face, GL_STATIC_DRAW );

		// 頂点バッファオブジェクトの結合を解除する
		glBindVertexArray( 0 );

		drawNormal( position, normal );
	}

	// 描画
	void draw()
	{
		//updatePosition();

		glBindVertexArray( vao );
		//glDrawArrays( GL_POINTS, 0, vertices );
		glDrawElements( GL_POINTS, indexes * 3, GL_UNSIGNED_INT, 0 );

		//glBindVertexArray( vao2 );
		//glDrawArrays( GL_LINES, 0, vertices * 2 );
	}

	// 法線の描画
	void drawNormal( Position *position, Normal *normal )
	{
		// 頂点配列オブジェクトを作成する
		glGenVertexArrays( 1, &vao2 );
		glBindVertexArray( vao2 );

		// 頂点バッファオブジェクトを作成する
		glGenBuffers( 1, &normalPointsBuffer );
		glBindBuffer( GL_ARRAY_BUFFER, normalPointsBuffer );
		glBufferData( GL_ARRAY_BUFFER, sizeof( Position ) * vertices * 2, NULL, GL_STATIC_DRAW );

		Position *normalPos = ( Position* )glMapBuffer( GL_ARRAY_BUFFER, GL_WRITE_ONLY );

		// 法線を描画する頂点の位置を設定する
		for ( int i = 0; i < vertices; ++i ){
				( *normalPos )[ 0 ] = ( *position )[ 0 ];
				( *normalPos )[ 1 ] = ( *position )[ 1 ];
				( *normalPos )[ 2 ] = ( *position )[ 2 ];

				++normalPos;

				( *normalPos )[ 0 ] = ( *position )[ 0 ] + ( *normal )[ 0 ] * 0.1;
				( *normalPos )[ 1 ] = ( *position )[ 1 ] + ( *normal )[ 1 ] * 0.1;
				( *normalPos )[ 2 ] = ( *position )[ 2 ] + ( *normal )[ 2 ] * 0.1;

				++normalPos;
				++position;
				++normal;
		}

		// 頂点バッファオブジェクトをattribute変数に対応づける
		glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, 0, 0 );
		// 0番のattrib変数を使用可能にする
		glEnableVertexAttribArray( 0 );

		glUnmapBuffer( GL_ARRAY_BUFFER );
		glBindBuffer( GL_ARRAY_BUFFER, 0 );

		// 頂点バッファオブジェクトの結合を解除する
		glBindVertexArray( 0 );
	}

	// 頂点位置の更新
	void updatePosition()
	{
		Position *hold;

		float t = ( float )frame / ( float )cycle;

		// 頂点バッファオブジェクトのメモリをプログラムのメモリ空間にマップする
		position = ( Position* )glMapBuffer( GL_ARRAY_BUFFER, GL_WRITE_ONLY );
		hold = position;

		for ( int j = 0; j < stacks; ++j ){
			for ( int i = 0; i < slices; ++i ){
				const auto x( ( GLfloat( i ) / GLfloat( slices - 1 ) - 0.5f ) * GLfloat( slices ) / GLfloat( stacks ) );
				const auto y( ( GLfloat( j ) / GLfloat( stacks - 1 ) - 0.5f ) );
				const auto r( hypot( x, y ) * 6.0f * pi );

				( *position )[ 0 ] = x;
				( *position )[ 1 ] = y;
				( *position )[ 2 ] = sin( r - 2.0f * pi * t ) / ( r + pi );
				++position;
			}
		}

		// 頂点バッファオブジェクトのメモリをプログラムのメモリ空間から切り離す
		glUnmapBuffer( GL_ARRAY_BUFFER );

		glBindBuffer( GL_ARRAY_BUFFER, positionBuffer );
		glBufferSubData( GL_ARRAY_BUFFER, 0, sizeof( Position ) * vertices, hold );
		if ( ++frame >= cycle ) frame = 0;
	}
	
};