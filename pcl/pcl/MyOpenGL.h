#pragma once
#define pi	3.14159265f	// 円周率
#include <iostream>
#include <vector>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

namespace MyOpenGL{
	// シェーダオブジェクトのコンパイル結果を表示する
	// shader: シェーダオブジェクト名
	// str: コンパイルエラーが発生した場所を示す文字列
	GLboolean printShaderInfoLog( GLuint shader, const char *str )
	{
		// コンパイル結果を取得する
		GLint status;
		glGetShaderiv( shader, GL_COMPILE_STATUS, &status );
		if ( status == GL_FALSE ) std::cerr << "Compile Error in " << str << std::endl;

		// シェーダのコンパイル時のログの長さを取得する
		GLsizei bufSize;
		glGetShaderiv( shader, GL_INFO_LOG_LENGTH, &bufSize );
		if ( bufSize > 1 ){
			// シェーダのコンパイル時のログの内容を取得する
			std::vector<GLchar> infoLog( bufSize );
			GLsizei length;
			glGetShaderInfoLog( shader, bufSize, &length, &infoLog[ 0 ] );
			std::cerr << &infoLog[ 0 ] << std::endl;
		}

		return static_cast<GLboolean>( status );
	}

	// プログラムオブジェクトのリンク結果を表示する
	// program: プログラムオブジェクト名
	GLboolean printProgramInfoLog( GLuint program )
	{
		// リンク結果を取得する
		GLint status;
		glGetProgramiv( program, GL_LINK_STATUS, &status );
		if ( status == GL_FALSE ) std::cerr << "Link Error." << std::endl;

		// シェーダのリンク時のログの長さを取得する
		GLsizei bufSize;
		glGetProgramiv( program, GL_INFO_LOG_LENGTH, &bufSize );

		if ( bufSize > 1 ){
			// シェーダのリンク時のログの内容を取得する
			std::vector<GLchar> infoLog( bufSize );
			GLsizei length;
			glGetProgramInfoLog( program, bufSize, &length, &infoLog[ 0 ] );
			std::cerr << &infoLog[ 0 ] << std::endl;
		}

		return static_cast<GLboolean>( status );
	}

	// プログラムオブジェクトを作成する
	// vsrc: バーテックスシェーダのソースプログラムの文字列
	// fsrc: フラグメントシェーダのソースプログラムの文字列
	GLuint createProgram( const char *vsrc, const char *fsrc )
	{
		// 空のプログラムオブジェクトを作成する
		const GLuint program( glCreateProgram() );

		if ( vsrc != NULL ){
			// バーテックスシェーダのシェーダオブジェクトを作成する
			const GLuint vobj( glCreateShader( GL_VERTEX_SHADER ) );
			glShaderSource( vobj, 1, &vsrc, NULL );
			glCompileShader( vobj );

			// バーテックスシェーダのシェーダオブジェクトをプログラムオブジェクトに組み込む
			if ( printShaderInfoLog( vobj, "vertex shader" ) ) glAttachShader( program, vobj );
			glDeleteShader( vobj );
		}

		if ( fsrc != NULL ){
			// フラグメントシェーダのシェーダオブジェクトを作成する
			const GLuint fobj( glCreateShader( GL_FRAGMENT_SHADER ) );
			glShaderSource( fobj, 1, &fsrc, NULL );
			glCompileShader( fobj );

			// フラグメントシェーダのシェーダオブジェクトをプログラムオブジェクトに組み込む
			if ( printShaderInfoLog( fobj, "fragment shader" ) ) glAttachShader( program, fobj );
			glDeleteShader( fobj );
		}

		// プログラムオブジェクトをリンクする
		glBindAttribLocation( program, 0, "position" );
		glBindAttribLocation( program, 1, "normal" );
		glBindFragDataLocation( program, 0, "fragment" );
		glLinkProgram( program );

		// 作成したプログラムオブジェクトを返す
		if ( printProgramInfoLog( program ) ) return program;

		// プログラムオブジェクトが作成できなければ 0 を返す
		glDeleteProgram( program );
		return 0;
	}

	// シェーダのソースファイルを読み込んだメモリを返す
	// name: シェーダのソースファイル名
	// buffer: 読み込んだソースファイルのテキスト
	bool readShaderSource( const char *name, std::vector<GLchar> &buffer )
	{
		// ファイル名がNULLだった
		if ( name == NULL )return false;

		// ソースファイルを開く
		std::ifstream file( name, std::ios::binary );
		if ( file.fail() ){
			// 開けなかった
			std::cerr << "Error: Can't open source file: " << name << std::endl;
			return false;
		}

		// ファイルの末尾に移動し現在位置（＝ファイルサイズ）を得る
		file.seekg( 0L, std::ios::end );
		GLsizei length = static_cast< GLsizei >( file.tellg() );

		// ファイルサイズのメモリを確保
		buffer.resize( length + 1 );

		// ファイルを先頭から読み込む
		file.seekg( 0L, std::ios::beg );
		file.read( buffer.data(), length );
		buffer[ length ] = '\0';

		if ( file.fail() ){
			// うまく読み込めなかった
			std::cerr << "Error: Could not read source file: " << name << std::endl;
			file.close();
			return false;
		}

		// 読み込み成功
		file.close();
		return true;
	}

	// シェーダのソースファイルを読み込んでプログラムオブジェクトを作成する
	// vert: バーテックスシェーダのソースファイル名
	// frag: フラグメントシェーダのソースファイル名
	GLuint loadProgram( const char *vert, const char *frag )
	{
		// シェーダのソースファイルを読み込む
		std::vector<GLchar> vsrc;
		const bool vstat( readShaderSource( vert, vsrc ) );
		std::vector<GLchar> fsrc;
		const bool fstat( readShaderSource( frag, fsrc ) );
		// プログラムオブジェクトを作成する
		return vstat && fstat ? createProgram( vsrc.data(), fsrc.data() ) : 0;
	}

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

	// 要素の内積
	// a: 3要素の配列
	// b: 3要素の配列
	inline GLfloat dot3( const GLfloat *a, const GLfloat *b )
	{
		return a[ 0 ] * b[ 0 ] + a[ 1 ] * b[ 1 ] + a[ 2 ] * b[ 2 ];
	}

	// 要素の外積
	// a: 3要素の配列
	// b: 3要素の配列
	// c: 結果を格納する3要素の配列
	inline void cross( GLfloat *c, const GLfloat *a, const GLfloat *b )
	{
		c[ 0 ] = a[ 1 ] * b[ 2 ] - a[ 2 ] * b[ 1 ];
		c[ 1 ] = a[ 2 ] * b[ 0 ] - a[ 0 ] * b[ 2 ];
		c[ 2 ] = a[ 0 ] * b[ 1 ] - a[ 1 ] * b[ 0 ];
	}

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
			// 文字列を読み込む
			std::istringstream str( line );
			// 文字列を展開
			std::string op;
			str >> op;

			// 頂点データの読み込み
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
			// 面データの読み込み
			else if ( op == "f" ){
				// 面データ
				index f;

				// 頂点座標番号を取り出す
				for ( int i = 0; i < 3; ++i ){
					// １行をスペースで区切って個々の要素の最小の数値を取り出す
					std::string s;
					str >> s;
					f.position[ i ] = atoi( s.c_str() );
				}

				// 面データを保存する
				tface.push_back( f );
			}
		}

		// ファイルの読み込みチェック
		if ( file.bad() ){
			// うまく読み込めなかった
			std::cerr << "Warning: Can't read OBJ file; " << name << std::endl;
		}

		// ファイルを閉じる
		file.close();

		// メモリの確保
		pos = norm = NULL;
		face = NULL;
		vertexNum = static_cast< GLuint >( tpos.size() );
		faceNum = static_cast< GLuint >( tface.size() );
		try{
			pos = new GLfloat[ vertexNum ][ 3 ];
			norm = new GLfloat[ vertexNum ][ 3 ];
			face = new GLuint[ faceNum ][ 3 ];
		}
		catch ( std::bad_alloc e ){
			delete[] pos;
			delete[] norm;
			delete[] face;

			pos = norm = NULL;
			face = NULL;

			return false;
		}

		// 位置と大きさの正規化のための係数
		GLfloat scale, centerX, centerY, centerZ;
		if ( normalize ){
			const float sx( xmax - xmin );
			const float sy( ymax - ymin );
			const float sz( zmax - zmin );

			scale = sx;
			if ( sy > scale ) scale = sy;
			if ( sz > scale ) scale = sz;
			scale = ( scale != 0.0f ) ? 2.0f / scale : 1.0f;

			centerX = ( xmax + xmin ) * 0.5;
			centerY = ( ymax + ymin ) * 0.5;
			centerZ = ( zmax + zmin ) * 0.5;
		}
		else{
			scale = 1.0f;
			centerX = centerY = centerZ = 0.0f;
		}

		// 図形の大きさと一の正規化とデータのコピー
		for ( std::vector<vec3f>::const_iterator it = tpos.begin(); it != tpos.end(); ++it ){
			const size_t v = it - tpos.begin();

			pos[ v ][ 0 ] = ( it->x - centerX ) * scale;
			pos[ v ][ 1 ] = ( it->y - centerY ) * scale;
			pos[ v ][ 2 ] = ( it->z - centerZ ) * scale;
		}

		// 頂点法線の値を０にしておく
		for ( GLuint i = 0; i < vertexNum; ++i ){
			norm[ i ][ 0 ] = norm[ i ][ 1 ] = norm[ i ][ 2 ] = 0.0f;
		}

		// 面の法線の算出とデータのコピー
		for ( std::vector<index>::const_iterator it = tface.begin(); it != tface.end(); ++it ){
			const size_t f( it - tface.begin() );

			// 頂点座標番号を取り出す
			const GLuint v0( face[ f ][ 0 ] = it->position[ 0 ] - 1 );
			const GLuint v1( face[ f ][ 1 ] = it->position[ 1 ] - 1 );
			const GLuint v2( face[ f ][ 2 ] = it->position[ 2 ] - 1 );

			// v1 - v0, v2 - v0 を求める
			const GLfloat d1[] = {
				pos[ v1 ][ 0 ] - pos[ v0 ][ 0 ],
				pos[ v1 ][ 1 ] - pos[ v0 ][ 1 ],
				pos[ v1 ][ 2 ] - pos[ v0 ][ 2 ]
			};
			const GLfloat d2[] = {
				pos[ v2 ][ 0 ] - pos[ v0 ][ 0 ],
				pos[ v2 ][ 1 ] - pos[ v0 ][ 1 ],
				pos[ v2 ][ 2 ] - pos[ v0 ][ 2 ]
			};

			// 外積により面法線を求める
			GLfloat n[ 3 ];
			cross( n, d1, d2 );

			// 面法線を頂点法線に積算する
			norm[ v0 ][ 0 ] += n[ 0 ];
			norm[ v0 ][ 1 ] += n[ 1 ];
			norm[ v0 ][ 2 ] += n[ 2 ];
			norm[ v1 ][ 0 ] += n[ 0 ];
			norm[ v1 ][ 1 ] += n[ 1 ];
			norm[ v1 ][ 2 ] += n[ 2 ];
			norm[ v2 ][ 0 ] += n[ 0 ];
			norm[ v2 ][ 1 ] += n[ 1 ];
			norm[ v2 ][ 2 ] += n[ 2 ];
		}

		// 頂点法線の正規化
		for ( GLuint v = 0; v < vertexNum; ++v ){
			// 頂点法線の長さ
			GLfloat a( sqrt( dot3( norm[ v ], norm[ v ] ) ) );

			// 頂点法線の正規化
			if ( a != 0.0f ){
				norm[ v ][ 0 ] /= a;
				norm[ v ][ 1 ] /= a;
				norm[ v ][ 2 ] /= a;
			}
		}

		return true;
	}

	// TGAファイルを読み込む
	// name:	読み込むファイル名
	// width:	読み込んだファイルの幅
	// height:	読み込んだファイルの高さ
	// format:	読み込んだファイルのフォーマット
	// return:	読み込んだ画像データのポインタ。読み込めなければnullptr
	GLubyte *loadTga( const char *name, GLsizei *width, GLsizei *height, GLenum *format )
	{
		// ファイルを開く
		std::ifstream file( name, std::ios::binary );

		// ファイルが開けなかったらもどる
		if ( !file )
		{
			std::cerr << "Error: Can't open file: " << name << std::endl;
			return nullptr;
		}

		// ヘッダを読み込む
		unsigned char header[ 18 ];
		file.read( reinterpret_cast< char* >( header ), sizeof header );

		// ヘッダの読み込みに失敗したらもどる
		if ( file.bad() )
		{
			std::cerr << "Error: Can't read file header: " << name << std::endl;
			file.close();
			return nullptr;
		}

		// 幅と高さ
		*width = header[ 13 ] << 8 | header[ 12 ];
		*height = header[ 15 ] << 8 | header[ 14 ];

		// 深度
		const size_t depth( header[ 16 ] / 8 );
		switch ( depth )
		{
			case 1:
				*format = GL_RED;
				break;
			case 2:
				*format = GL_RG;
				break;
			case 3:
				*format = GL_BGR;
			case 4:
				*format = GL_BGRA;
				break;
			default:
				// 存在しないフォーマットなら戻る
				std::cerr << "Error: Unusable format: " << depth << std::endl;
				file.close();
				return nullptr;
		}

		// データサイズ
		const size_t size( *width * *height * depth );

		// 読み込みに使うメモリを確保する
		GLubyte *const buffer( new( std::nothrow )GLubyte[ size ] );

		// メモリが確保できなければもどる
		if ( buffer == nullptr )
		{
			std::cerr << "Error: Too large file: " << name << std::endl;
			file.close();
			return nullptr;
		}

		// データを読み込む
		if ( header[ 2 ] & 8 )
		{
			// RLE
			size_t p( 0 );
			char c;
			while ( file.get( c ) )
			{
				if ( c & 0x80 )
				{
					// run-length packet
					const size_t count( ( c & 0x7f ) + 1 );
					if ( p + count * depth > size )break;
					char tmp[ 4 ];
					file.read( tmp, depth );
					for ( size_t i = 0; i < count; ++i )
					{
						for ( size_t j = 0; j < depth; ) buffer[ p++ ] = tmp[ j++ ];
					}
				}
				else
				{
					// raw packet
					const size_t count( ( c + 1 ) * depth );
					if ( p + count > size )break;
					file.read( reinterpret_cast< char * >( buffer + p ), count );
					p += count;
				}
			}
		}
		else
		{
			// 非圧縮
			file.read( reinterpret_cast< char * >( buffer ), size );
		}

		// 読み込みに失敗していたら警告を出す
		if ( file.bad() )
		{
			std::cerr << "Warning: Can't read image data: " << name << std::endl;
		}

		// ファイルを閉じる
		file.close();

		// 画像を読み込んだメモリを返す
		return buffer;
	}

	// 配列に格納された画像のないようをTGAファイルに保存する
	// width:	画像の幅
	// height:	画像の高さ
	// depth:	画像の１画素のバイト数
	// buffer:	画像データ
	// name:	ファイル名
	// return:	保存に成功したら
	bool saveTga( GLsizei width, GLsizei height, unsigned int depth, const void *buffer, const char *name )
	{
		// ファイルを開く
		std::ofstream file( name, std::ios::binary );

		// ファイルが開けなかったらもどる
		if ( !file )
		{
			std::cerr << "Error: Can't open file: " << name << std::endl;
			return false;
		}

		// 画像のヘッダ
		const unsigned char type( depth == 0 ? 0 : depth < 3 ? 3 : 2 );
		const unsigned char alpha( depth == 2 || depth == 4 ? 8 : 0 );
		const unsigned char header[ 18 ] =
		{
			0,		// ID length
			0,		// Color map type (none)
			type,	// Image Type (2:RGB, 3:Grayscale)
			0, 0,	// Offset into the color map table
			0, 0,	// Number of color map entries
			0,		// Number of a color map entry bits per pixel
			0, 0,	// Horizontal image position
			0, 0,	// Vertical image position
			( unsigned char )( width & 0xff ),
			( unsigned char )( width >> 8 ),
			( unsigned char )( height & 0xff ),
			( unsigned char )( height >> 8 ),
			( unsigned char )( depth * 8 ),	// Pixel depth(bits per pixel)
			alpha	// Image descriptor
		};

		// ヘッダを書き込む
		file.write( reinterpret_cast< const char* >( header ), sizeof header );

		// ヘッダの書き込みチェック
		if ( file.bad() )
		{
			// ヘッダの書き込みに失敗した
			std::cerr << "Error: Can't write file header: " << name << std::endl;
			file.close();
			return false;
		}

		// データを書き込む
		size_t size( width * height * depth );
		if ( type == 2 )
		{
			// フルカラー
			std::vector<char> temp( size );
			for ( size_t i = 0; i < size; i += depth )
			{
				temp[ i + 2 ] = static_cast< const char* >( buffer )[ i + 0 ];
				temp[ i + 1 ] = static_cast< const char* >( buffer )[ i + 1 ];
				temp[ i + 0 ] = static_cast< const char* >( buffer )[ i + 2 ];
				if ( depth == 4 )temp[ i + 3 ] = static_cast< const char* >( buffer )[ i + 3 ];
			}
			file.write( &temp[ 0 ], size );
		}
		else if ( type == 3 )
		{
			// グレースケール
			file.write( static_cast< const char* >( buffer ), size );
		}

		// フッタを書き込む
		static const char footer[] = "\0\0\0\0\0\0\0\0TRUEVISION-XFILE.";
		file.write( footer, sizeof footer );

		// データの書き込みチェック
		if ( file.bad() )
		{
			// データの書き込みに失敗した
			std::cerr << "Error: Can't write image data: " << name << std::endl;
			file.close();
			return false;
		}

		// ファイルを閉じる
		file.close();

		return true;
	}

	// 放射照度マップの作成
	bool createMap( const char *name, GLsizei diameter, GLuint imap, GLsizei isize, 
		GLuint emap, GLsizei esize, const GLfloat *amb, GLfloat shi )
	{
		// 作成したテクスチャの数
		static int count( 0 );

		// 読み込んだ画像の幅と高さ、フォーマット
		GLsizei width, height;
		GLenum format;

		// 天空画像のファイルの読み込み
		GLubyte const *const texture( loadTga( name, &width, &height, &format ) );

		// 画像が読み込めなければ終了
		if ( !texture )return false;

		// この画像の中心位置
		const GLsizei centerX( width / 2 ), centerY( height / 2 );

		// diameter, width, heightの最小値の1/2をraduisにする
		const GLsizei radius( std::min( diameter, std::min( width, height ) ) / 2 );

		// 平滑化した放射照度マップの一時保存先
		std::vector<GLubyte> itemp( isize * isize * 3 );

		// 放射照度マップ用に平滑する
		//smooth( texture, width, height, format, centerX, centerY, radius, radius, &itemp[ 0 ], isize, amb, 1.0f );

		// 作成したテクスチャを保存する
		std::stringstream imapname;
		imapname << "irr" << count << ".tga";
		saveTga( isize, isize, 3, &itemp[ 0 ], imapname.str().c_str() );

		// 平滑した環境マップの一時保存先
		std::vector<GLubyte> etemp( esize * esize * 3 );

		// 環境マップ用に平滑する
		//smooth( texture, width, height, format, centerX, centerY, radius, radius, &etemp[ 0 ], esize, amb, shi );

		// 環境マップのテクスチャを作成する
		std::stringstream emapname;
		emapname << "env" << count << ".tga";
		saveTga( esize, esize, 3, &etemp[ 0 ], emapname.str().c_str() );

		// 読み込んだデータはもう使わないのでメモリを開放する
		delete[] texture;

		// 作成したテクスチャの数を数える
		++count;

		return true;
	}
}