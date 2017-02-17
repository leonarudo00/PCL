#pragma once

//
// 球
//

// 自作ヘッダ
#include "MyOpenGL.h"

class Sphere
{
	// 頂点配列オブジェクト
	GLuint vao;

	// 頂点バッファオブジェクト
	GLuint vbo[ 2 ];

	typedef GLfloat Position[ 3 ];
	typedef GLfloat Edge[ 2 ];

	// 頂点の数
	GLint vertices;

public:

	// コンストラクタ
	Sphere( int slices, int stacks )
		:vertices( slices * ( stacks - 1 ) + 2 )
	{
		// 頂点バッファオブジェクトのメモリを参照するポインタ
		Position *position;
		Edge *edge;

		// 頂点、稜線の数を求める
		GLuint vertices = slices * ( stacks - 1 ) + 2;
		GLuint edges = slices * ( stacks - 1 ) * 2 + slices;

		// 頂点配列オブジェクトを作成する
		glGenVertexArrays( 1, &vao );
		glBindVertexArray( vao );

		// 頂点バッファオブジェクトを作成する
		glGenBuffers( 2, vbo );
		glBindBuffer( GL_ARRAY_BUFFER, vbo[ 0 ] );
		glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, vbo[ 1 ] );

		// 頂点バッファオブジェクトにメモリ領域を確保する
		glBufferData( GL_ARRAY_BUFFER, sizeof( Position ) * vertices, NULL, GL_STATIC_DRAW );
		glBufferData( GL_ELEMENT_ARRAY_BUFFER, sizeof( Edge ) * edges, NULL, GL_STATIC_DRAW );

		// 頂点バッファオブジェクトのメモリをプログラムのメモリ空間にマップする
		position = ( Position * )glMapBuffer( GL_ARRAY_BUFFER, GL_WRITE_ONLY );
		edge = ( Edge * )glMapBuffer( GL_ELEMENT_ARRAY_BUFFER, GL_WRITE_ONLY );

		// 北極点の位置
		( *position )[ 0 ] = 0.0f;
		( *position )[ 1 ] = 1.0f;
		( *position )[ 2 ] = 0.0f;
		++position;

		// 中間部分の頂点の位置
		for ( int j = 1; j < stacks; ++j ){
			float ph = 3.141593f * ( float )j / ( float )stacks;
			float y = cosf( ph );
			float r = sinf( ph );

			for ( int i = 0; i < slices; ++i ){
				float th = 2.0f * 3.141593f * ( float )i / ( float )slices;
				float x = r * cosf( th );
				float z = r * sinf( th );

				( *position )[ 0 ] = x;
				( *position )[ 1 ] = y;
				( *position )[ 2 ] = z;
				++position;
			}
		}

		// 南極点の位置
		( *position )[ 0 ] = 0.0f;
		( *position )[ 1 ] = -1.0f;
		( *position )[ 2 ] = 0.0f;

		int count;

		// 北極点周りの稜線
		for ( int i = 1; i <= slices; ++i ){
			( *edge )[ 0 ] = 0;
			( *edge )[ 1 ] = i;
			++edge;
		}

		// 中間部分の稜線
		count = 1;
		for ( int j = 2; j < stacks; ++j ){
			for ( int i = 1; i < slices; ++i ){
				// 右方向
				( *edge )[ 0 ] = count;
				( *edge )[ 1 ] = count + 1;
				++edge;

				// 下方向
				( *edge )[ 0 ] = count;
				( *edge )[ 1 ] = count + slices;

				++count;
			}

			// 右端の右方向の稜線は左端の頂点に接続する
			( *edge )[ 0 ] = count;
			( *edge )[ 1 ] = count - slices + 1;
			++edge;
			
			// 下方向
			( *edge )[ 0 ] = count;
			( *edge )[ 1 ] = count + slices;
			++edge;

			++count;
		}

		// 最下段の稜線
		for ( int i = 1; i < slices; ++i ){
			// 右方向
			( *edge )[ 0 ] = count;
			( *edge )[ 1 ] = count + 1;
			++edge;

			// 下方向の稜線は南極点に接続する
			( *edge )[ 0 ] = count;
			( *edge )[ 1 ] = vertices - 1;
			++edge;

			++count;
		}

		// 右端の右方向の稜線は左端の頂点に接続する
		( *edge )[ 0 ] = count;
		( *edge )[ 1 ] = count - slices + 1;
		++edge;

		// 下方向の稜線は南極点に接続する
		( *edge )[ 0 ] = count;
		( *edge )[ 1 ] = vertices - 1;

		// 頂点バッファオブジェクトのメモリをプログラムのメモリ空間から切り離す
		glUnmapBuffer( GL_ELEMENT_ARRAY_BUFFER );
		glUnmapBuffer( GL_ARRAY_BUFFER );

		// 頂点バッファオブジェクトを解放する
		glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, 0 );
		glBindBuffer( GL_ARRAY_BUFFER, 0 );

		// 0 番の attribute 変数からデータを入力する
		glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, 0, 0 );
		glEnableVertexAttribArray( 0 );
	}

	// デストラクタ
	virtual ~Sphere()
	{
		// 頂点配列オブジェクトを削除する
		glDeleteVertexArrays( 1, &vao );

		// 頂点バッファオブジェクトを削除する
		glDeleteBuffers( 2, vbo );
	}

	// 描画
	virtual void draw() const
	{
		// 頂点配列オブジェクトを指定して描画する
		glBindVertexArray( vao );
		glDrawArrays( GL_LINES, 0, vertices );
	}
};