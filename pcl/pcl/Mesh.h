#pragma once

//
// ���b�V��
//

// ����w�b�_
#include "MyOpenGL.h"

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/integral_image_normal.h>

class Mesh
{
	// ���_�o�b�t�@�I�u�W�F�N�g�̃��������Q�Ƃ���|�C���^
	typedef GLfloat Position[ 3 ];
	typedef GLfloat Normal[ 3 ];
	typedef GLuint Face[ 3 ];
	Position *position;
	Normal *normal;
	Face *face;

	GLuint	vertices;			// ���_��

	GLuint	indexes;			// �C���f�b�N�X�̐�

	GLuint	vao;				// ���_�z��I�u�W�F�N�g

	GLuint	vao2;
	
	GLuint	positionBuffer;		// ���_�ʒu���i�[���钸�_�o�b�t�@�I�u�W�F�N�g

	GLuint	indexBuffer;		// ���_�C���f�b�N�X���i�[���钸�_�o�b�t�@�I�u�W�F�N�g

	GLuint	normalBuffer;		// ���_�@�����i�[���钸�_�o�b�t�@�I�u�W�F�N�g

	GLuint	normalPointsBuffer;	// �@����`�悷�邽�߂̒��_�ʒu���i�[���钸�_�o�b�t�@�I�u�W�F�N�g

	int		frame = 0;			// �t���[��

	int		cycle = 100;		// ���g��

	int		slices;				// ���̒��_��

	int		stacks;				// �c�̒��_��

public:
	// �R���X�g���N�^
	// slices: ���̒��_��
	// stacks: �c�̒��_��
	Mesh( int slices, int stacks )
		:vertices( slices * stacks )
		, indexes( ( slices - 1 )*( stacks - 1 ) * 2 * 3 )
		, slices( slices )
		, stacks( stacks )
	{

		// ���_�z��I�u�W�F�N�g���쐬����
		glGenVertexArrays( 1, &vao );
		glBindVertexArray( vao );

		// ���_�o�b�t�@�I�u�W�F�N�g���쐬����
		glGenBuffers( 1, &positionBuffer );
		glBindBuffer( GL_ARRAY_BUFFER, positionBuffer );
		glGenBuffers( 1, &indexBuffer );
		glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, indexBuffer );

		// ���_�o�b�t�@�I�u�W�F�N�g�Ƀ������̈���m�ۂ���
		glBufferData( GL_ARRAY_BUFFER, sizeof( Position ) * vertices, NULL, GL_DYNAMIC_DRAW );
		glBufferData( GL_ELEMENT_ARRAY_BUFFER, sizeof( Face ) * indexes, NULL, GL_STATIC_DRAW );

		// ���_�o�b�t�@�I�u�W�F�N�g�̃��������v���O�����̃�������ԂɃ}�b�v����
		position = ( Position* )glMapBuffer( GL_ARRAY_BUFFER, GL_WRITE_ONLY );
		face = ( Face* )glMapBuffer( GL_ELEMENT_ARRAY_BUFFER, GL_WRITE_ONLY );

		// ���_�ʒu��ݒ肷��
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

		// ���_�o�b�t�@�I�u�W�F�N�g�̃��������v���O�����̃�������Ԃ���؂藣��
		glUnmapBuffer( GL_ARRAY_BUFFER );

		// ���_�o�b�t�@�I�u�W�F�N�g��attribute�ϐ��ɑΉ��Â���
		glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, 0, 0 );
		// 0�Ԃ�attrib�ϐ����g�p�\�ɂ���
		glEnableVertexAttribArray( 0 );

		// �C���f�b�N�X��ݒ肷��
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

		// ���_�o�b�t�@�I�u�W�F�N�g�̃��������v���O�����̃�������Ԃ���؂藣��
		glUnmapBuffer( GL_ELEMENT_ARRAY_BUFFER );

		// ���_�o�b�t�@�I�u�W�F�N�g�̌�������������
		glBindVertexArray( 0 );

	}
	// �R���X�g���N�^
	// name: �t�@�C����
	// normalize: ���K���̗L��
	Mesh( const char *name, bool normalize )
	{
		// OBJ�t�@�C����ǂݍ���
		MyOpenGL::loadOBJ( name, vertices, position, normal, indexes, face, normalize );

		// PCL��OBJ�t�@�C����ǂݍ���
		pcl::PolygonMesh::Ptr mesh( new pcl::PolygonMesh() );
		pcl::PointCloud < pcl::PointXYZ >::Ptr obj_pcd( new pcl::PointCloud<pcl::PointXYZ>() );
		if ( pcl::io::loadPolygonFileOBJ( name, *mesh ) != -1 ){
			pcl::fromPCLPointCloud2( mesh->cloud, *obj_pcd );
		}

		// ���_�z��I�u�W�F�N�g���쐬����
		glGenVertexArrays( 1, &vao );
		glBindVertexArray( vao );

		// ���_�o�b�t�@�I�u�W�F�N�g���쐬����
		glGenBuffers( 1, &positionBuffer );
		glBindBuffer( GL_ARRAY_BUFFER, positionBuffer );
		glGenBuffers( 1, &normalBuffer );
		glBindBuffer( GL_ARRAY_BUFFER, normalBuffer );
		glGenBuffers( 1, &indexBuffer );
		glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, indexBuffer );

		// ���_�ʒu�o�b�t�@�I�u�W�F�N�g�Ƀ������̈���m�ۂ���
		glBufferData( GL_ARRAY_BUFFER, sizeof( Position ) * vertices, NULL, GL_DYNAMIC_DRAW );
		// ���_�o�b�t�@�I�u�W�F�N�g�̃��������v���O�����̃�������ԂɃ}�b�v����
		position = ( Position* )glMapBuffer( GL_ARRAY_BUFFER, GL_WRITE_ONLY );
		// �@����`�悷�钸�_�̈ʒu��ݒ肷��
		for ( int i = 0; i < vertices; ++i ){
			( *position )[ 0 ] = obj_pcd->points[ i ].x;
			( *position )[ 1 ] = obj_pcd->points[ i ].y;
			( *position )[ 2 ] = obj_pcd->points[ i ].z;
			++position;
		}
		// attribute�ϐ��ɑΉ��Â���
		glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, 0, 0 );
		// 0�Ԃ�attrib�ϐ����g�p�\�ɂ���
		glEnableVertexAttribArray( 0 );
		// ��������Ԃ���؂藣��
		glUnmapBuffer( GL_ARRAY_BUFFER );
		glBindBuffer( GL_ARRAY_BUFFER, 0 );

		// ���_�@���ʒu�o�b�t�@�I�u�W�F�N�g�Ƀ������̈���m�ۂ���
		glBufferData( GL_ARRAY_BUFFER, sizeof( Normal ) * vertices, normal, GL_DYNAMIC_DRAW );
		// attribute�ϐ��ɑΉ��Â���
		glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, 0, 0 );
		// 1�Ԃ�attrib�ϐ����g�p�\�ɂ���
		glEnableVertexAttribArray( 1 );
		// ��������Ԃ���؂藣��
		glUnmapBuffer( GL_ARRAY_BUFFER );
		glBindBuffer( GL_ARRAY_BUFFER, 0 );

		// ���_�C���f�b�N�X�o�b�t�@�I�u�W�F�N�g�Ƀ������̈���m�ۂ���
		glBufferData( GL_ELEMENT_ARRAY_BUFFER, sizeof( Face ) * indexes, face, GL_STATIC_DRAW );

		// ���_�o�b�t�@�I�u�W�F�N�g�̌�������������
		glBindVertexArray( 0 );

		drawNormal( position, normal );
	}

	// �`��
	void draw()
	{
		//updatePosition();

		glBindVertexArray( vao );
		//glDrawArrays( GL_POINTS, 0, vertices );
		glDrawElements( GL_POINTS, indexes * 3, GL_UNSIGNED_INT, 0 );

		//glBindVertexArray( vao2 );
		//glDrawArrays( GL_LINES, 0, vertices * 2 );
	}

	// �@���̕`��
	void drawNormal( Position *position, Normal *normal )
	{
		// ���_�z��I�u�W�F�N�g���쐬����
		glGenVertexArrays( 1, &vao2 );
		glBindVertexArray( vao2 );

		// ���_�o�b�t�@�I�u�W�F�N�g���쐬����
		glGenBuffers( 1, &normalPointsBuffer );
		glBindBuffer( GL_ARRAY_BUFFER, normalPointsBuffer );
		glBufferData( GL_ARRAY_BUFFER, sizeof( Position ) * vertices * 2, NULL, GL_STATIC_DRAW );

		Position *normalPos = ( Position* )glMapBuffer( GL_ARRAY_BUFFER, GL_WRITE_ONLY );

		// �@����`�悷�钸�_�̈ʒu��ݒ肷��
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

		// ���_�o�b�t�@�I�u�W�F�N�g��attribute�ϐ��ɑΉ��Â���
		glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, 0, 0 );
		// 0�Ԃ�attrib�ϐ����g�p�\�ɂ���
		glEnableVertexAttribArray( 0 );

		glUnmapBuffer( GL_ARRAY_BUFFER );
		glBindBuffer( GL_ARRAY_BUFFER, 0 );

		// ���_�o�b�t�@�I�u�W�F�N�g�̌�������������
		glBindVertexArray( 0 );
	}

	// ���_�ʒu�̍X�V
	void updatePosition()
	{
		Position *hold;

		float t = ( float )frame / ( float )cycle;

		// ���_�o�b�t�@�I�u�W�F�N�g�̃��������v���O�����̃�������ԂɃ}�b�v����
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

		// ���_�o�b�t�@�I�u�W�F�N�g�̃��������v���O�����̃�������Ԃ���؂藣��
		glUnmapBuffer( GL_ARRAY_BUFFER );

		glBindBuffer( GL_ARRAY_BUFFER, positionBuffer );
		glBufferSubData( GL_ARRAY_BUFFER, 0, sizeof( Position ) * vertices, hold );
		if ( ++frame >= cycle ) frame = 0;
	}
	
};