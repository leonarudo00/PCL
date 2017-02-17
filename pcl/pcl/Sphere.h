#pragma once

//
// ��
//

// ����w�b�_
#include "MyOpenGL.h"

class Sphere
{
	// ���_�z��I�u�W�F�N�g
	GLuint vao;

	// ���_�o�b�t�@�I�u�W�F�N�g
	GLuint vbo[ 2 ];

	typedef GLfloat Position[ 3 ];
	typedef GLfloat Edge[ 2 ];

	// ���_�̐�
	GLint vertices;

public:

	// �R���X�g���N�^
	Sphere( int slices, int stacks )
		:vertices( slices * ( stacks - 1 ) + 2 )
	{
		// ���_�o�b�t�@�I�u�W�F�N�g�̃��������Q�Ƃ���|�C���^
		Position *position;
		Edge *edge;

		// ���_�A�Ő��̐������߂�
		GLuint vertices = slices * ( stacks - 1 ) + 2;
		GLuint edges = slices * ( stacks - 1 ) * 2 + slices;

		// ���_�z��I�u�W�F�N�g���쐬����
		glGenVertexArrays( 1, &vao );
		glBindVertexArray( vao );

		// ���_�o�b�t�@�I�u�W�F�N�g���쐬����
		glGenBuffers( 2, vbo );
		glBindBuffer( GL_ARRAY_BUFFER, vbo[ 0 ] );
		glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, vbo[ 1 ] );

		// ���_�o�b�t�@�I�u�W�F�N�g�Ƀ������̈���m�ۂ���
		glBufferData( GL_ARRAY_BUFFER, sizeof( Position ) * vertices, NULL, GL_STATIC_DRAW );
		glBufferData( GL_ELEMENT_ARRAY_BUFFER, sizeof( Edge ) * edges, NULL, GL_STATIC_DRAW );

		// ���_�o�b�t�@�I�u�W�F�N�g�̃��������v���O�����̃�������ԂɃ}�b�v����
		position = ( Position * )glMapBuffer( GL_ARRAY_BUFFER, GL_WRITE_ONLY );
		edge = ( Edge * )glMapBuffer( GL_ELEMENT_ARRAY_BUFFER, GL_WRITE_ONLY );

		// �k�ɓ_�̈ʒu
		( *position )[ 0 ] = 0.0f;
		( *position )[ 1 ] = 1.0f;
		( *position )[ 2 ] = 0.0f;
		++position;

		// ���ԕ����̒��_�̈ʒu
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

		// ��ɓ_�̈ʒu
		( *position )[ 0 ] = 0.0f;
		( *position )[ 1 ] = -1.0f;
		( *position )[ 2 ] = 0.0f;

		int count;

		// �k�ɓ_����̗Ő�
		for ( int i = 1; i <= slices; ++i ){
			( *edge )[ 0 ] = 0;
			( *edge )[ 1 ] = i;
			++edge;
		}

		// ���ԕ����̗Ő�
		count = 1;
		for ( int j = 2; j < stacks; ++j ){
			for ( int i = 1; i < slices; ++i ){
				// �E����
				( *edge )[ 0 ] = count;
				( *edge )[ 1 ] = count + 1;
				++edge;

				// ������
				( *edge )[ 0 ] = count;
				( *edge )[ 1 ] = count + slices;

				++count;
			}

			// �E�[�̉E�����̗Ő��͍��[�̒��_�ɐڑ�����
			( *edge )[ 0 ] = count;
			( *edge )[ 1 ] = count - slices + 1;
			++edge;
			
			// ������
			( *edge )[ 0 ] = count;
			( *edge )[ 1 ] = count + slices;
			++edge;

			++count;
		}

		// �ŉ��i�̗Ő�
		for ( int i = 1; i < slices; ++i ){
			// �E����
			( *edge )[ 0 ] = count;
			( *edge )[ 1 ] = count + 1;
			++edge;

			// �������̗Ő��͓�ɓ_�ɐڑ�����
			( *edge )[ 0 ] = count;
			( *edge )[ 1 ] = vertices - 1;
			++edge;

			++count;
		}

		// �E�[�̉E�����̗Ő��͍��[�̒��_�ɐڑ�����
		( *edge )[ 0 ] = count;
		( *edge )[ 1 ] = count - slices + 1;
		++edge;

		// �������̗Ő��͓�ɓ_�ɐڑ�����
		( *edge )[ 0 ] = count;
		( *edge )[ 1 ] = vertices - 1;

		// ���_�o�b�t�@�I�u�W�F�N�g�̃��������v���O�����̃�������Ԃ���؂藣��
		glUnmapBuffer( GL_ELEMENT_ARRAY_BUFFER );
		glUnmapBuffer( GL_ARRAY_BUFFER );

		// ���_�o�b�t�@�I�u�W�F�N�g���������
		glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, 0 );
		glBindBuffer( GL_ARRAY_BUFFER, 0 );

		// 0 �Ԃ� attribute �ϐ�����f�[�^����͂���
		glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, 0, 0 );
		glEnableVertexAttribArray( 0 );
	}

	// �f�X�g���N�^
	virtual ~Sphere()
	{
		// ���_�z��I�u�W�F�N�g���폜����
		glDeleteVertexArrays( 1, &vao );

		// ���_�o�b�t�@�I�u�W�F�N�g���폜����
		glDeleteBuffers( 2, vbo );
	}

	// �`��
	virtual void draw() const
	{
		// ���_�z��I�u�W�F�N�g���w�肵�ĕ`�悷��
		glBindVertexArray( vao );
		glDrawArrays( GL_LINES, 0, vertices );
	}
};