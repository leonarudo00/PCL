#pragma once
#include <GL/glew.h>

// 図形データ
class Object
{
	// 頂点配列オブジェクト名
	GLuint vao;

	// 頂点バッファオブジェクト名
	GLuint vbo[ 2 ];

	// 頂点バッファオブジェクトのメモリを参照するポインタ
	typedef GLfloat Position[ 3 ];
	Position *poosition;
	typedef GLuint Edge[ 2 ];
	Edge *edge;

public:
	// 頂点属性
	struct Vertex
	{
		// 位置
		GLfloat position[ 2 ];
	};

	// コンストラクタ
	// size: 頂点の位置の次元
	// vertexcount: 頂点の数
	// vertex: 頂点属性を格納した配列
	Object( GLint size, GLuint vertexcount, const Vertex *vertex ){
		// 頂点配列オブジェクト
		glGenVertexArrays( 1, &vao );
		glBindVertexArray( vao );

		// 頂点バッファオブジェクト
		glGenBuffers( 2, vbo );
		glBindBuffer( GL_ARRAY_BUFFER, vbo[0] );
		glBufferData( GL_ARRAY_BUFFER, vertexcount * sizeof( Vertex ), vertex, GL_STATIC_DRAW );

		// 結合されている頂点バッファオブジェクトをin変数から参照できるようにする
		glVertexAttribPointer( 0, size, GL_FLOAT, GL_FALSE, 0, 0 );
		glEnableVertexAttribArray( 0 );
	}

	// デストラクタ
	virtual ~Object(){
		// 頂点配列オブジェクトを削除する
		glDeleteBuffers( 1, &vao );

		// 頂点バッファオブジェクトを削除する
		glDeleteBuffers( 2, vbo );
	}

private:
	// コピーコンストラクタによるコピー禁止
	Object( const Object &o );

	// 代入によるコピー禁止
	Object &operator = ( const Object &o );

public:
	// 頂点配列オブジェクトの結合
	void bind() const{
		// 描画する頂点配列オブジェクトを指定する
		glBindVertexArray( vao );
	}
};