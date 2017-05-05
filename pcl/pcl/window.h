#pragma once
#include <iostream>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

// �E�B���h�E�֘A�̏���
class Window
{
	// �E�B���h�E�̃n���h��
	GLFWwindow *const window;

	// ���邳
	int blightness;

	// �L�[�{�[�h�̏��
	int keyStatus;

	// �c����
	GLfloat aspect;

	// ���[���h���W�n�ɑ΂���f�o�C�X���W�n�̊g�嗦
	GLfloat scale;

	// ���[���h���W�n�ɑ΂��鐳�K���f�o�C�X���W�n�̊g�嗦
	GLfloat normalizedScale[ 2 ];

	// �E�B���h�E�̃T�C�Y
	GLfloat size[ 2 ];

	// �}�`�̐��K���f�o�C�X���W�n��ł̈ʒu
	GLfloat location[ 2 ];

	// ���[���h���W�n�ɑ΂��鐳�K���f�o�C�X���W�n�̊g�嗦���X�V����
	void updateScale()
	{
		normalizedScale[ 0 ] = scale * 2.0f / static_cast<GLfloat>( size[ 0 ] );
		normalizedScale[ 1 ] = scale * 2.0f / static_cast<GLfloat>( size[ 1 ] );
	}

public:
	// �R���X�g���N�^
	Window( int width = 640, int height = 480, const char *title = "Hello!" )
		: window( glfwCreateWindow( width, height, title, NULL, NULL ) )
		, scale( 100.0f )
		, keyStatus( GLFW_RELEASE )
	{
		if ( window == NULL )
		{
			// �E�B���h�E���쐬�ł��Ȃ�����
			std::cerr << "Can't create GLFW window." << std::endl;
			glfwTerminate();
			return;
		}

		// �쐬�����E�B���h�E�� OpenGL �̏����Ώۂɂ���
		glfwMakeContextCurrent( window );

		// GLEW ������������
		glewExperimental = GL_TRUE;
		if ( glewInit() != GLEW_OK )
		{
			// GLEW �̏������Ɏ��s����
			std::cerr << "Can't initialize GLEW" << std::endl;
			return;
		}

		// ���������̃^�C�~���O��҂�
		glfwSwapInterval( 1 );

		// ���̃C���X�^���X��this�|�C���^���L�^���Ă���
		glfwSetWindowUserPointer( window, this );

		// �E�B���h�E�̃T�C�Y�ύX���ɌĂяo�������̓o�^
		glfwSetWindowSizeCallback( window, resize );

		// �}�E�X�z�C�[�����쎞�ɌĂяo�������̓o�^
		glfwSetScrollCallback( window, wheel );

		// �L�[�{�[�h���쎞�ɌĂяo�������̓o�^
		glfwSetKeyCallback( window, keyboard );

		// �J�����E�B���h�E�̏����ݒ�
		resize( window, width, height );

		// �}�`�̐��K���f�o�C�X���W�n��ł̈ʒu�̏����l
		location[ 0 ] = location[ 1 ] = 0.0f;
	}

	// �f�X�g���N�^
	virtual ~Window()
	{
		glfwDestroyWindow( window );
	}

	// �E�B���h�E�����ׂ����𔻒肷��
	int shouldClose() const
	{
		return glfwWindowShouldClose( window ) || glfwGetKey( window, GLFW_KEY_ESCAPE );
	}

	// �J���[�o�b�t�@�����ւ��ăC�x���g�����o��
	void swapBuffers()
	{
		// �J���[�o�b�t�@�����ւ���
		glfwSwapBuffers( window );

		// �C�x���g�����o��
		glfwPollEvents();

		// �L�[�{�[�h�̏�Ԃ𒲂ׂ�
		if ( glfwGetKey( window, GLFW_KEY_LEFT ) != GLFW_RELEASE )	location[ 0 ] -= normalizedScale[ 0 ] / scale;
		else if ( glfwGetKey( window, GLFW_KEY_RIGHT ) != GLFW_RELEASE )	location[ 0 ] += normalizedScale[ 0 ] / scale;
		if ( glfwGetKey( window, GLFW_KEY_DOWN ) != GLFW_RELEASE )	location[ 1 ] -= normalizedScale[ 1 ] / scale;
		else if ( glfwGetKey( window, GLFW_KEY_UP ) != GLFW_RELEASE )	location[ 1 ] += normalizedScale[ 1 ] / scale;

		// �}�E�X���{�^���̏�Ԃ𒲂ׂ�
		if ( glfwGetMouseButton( window, GLFW_MOUSE_BUTTON_1 ) != GLFW_RELEASE )
		{
			// �}�E�X�J�[�\���̈ʒu���擾����
			double x, y;
			glfwGetCursorPos( window, &x, &y );

			// �}�E�X�J�[�\���̐��K���f�o�C�X���W�n��ł̈ʒu�����߂�
			location[ 0 ] = static_cast< GLfloat >( x )* 2.0f / size[ 0 ] - 1.0f;
			location[ 1 ] = 1.0f - static_cast< GLfloat >( y )* 2.0f / size[ 1 ];
		}
	}

	// �c��������o��
	GLfloat getAspect() const
	{
		return aspect;
	}

	// �E�B���h�E�̃T�C�Y�����o��
	const GLfloat *getSize() const
	{
		return size;
	}

	// ���[���h���W�n�ɑ΂���f�o�C�X���W�n�̊g�嗦�����o��
	GLfloat getScale() const
	{
		return scale;
	}

	// �ʒu�����o��
	const GLfloat *getLocation() const
	{
		return location;
	}

	// �E�B���h�E�̃T�C�Y�ύX���̏���
	static void resize( GLFWwindow *const window, int width, int height )
	{
		// �E�B���h�E�S�̂��r���[�|�[�g�ɐݒ肷��
		glViewport( 0, 0, width, height );

		// ���̃C���X�^���X��this�|�C���^�𓾂�
		Window *const instance( static_cast< Window* >( glfwGetWindowUserPointer( window ) ) );

		if ( instance != NULL )
		{
			// �J�����E�B���h�E�̃T�C�Y��ۑ�����
			instance->size[ 0 ] = width;
			instance->size[ 1 ] = height;

			// ���[���h���W�n�ɑ΂��鐳�K���f�o�C�X���W�n�̊g�嗦���X�V����
			instance->updateScale();
		}
	}

	// �}�E�X�z�C�[�����쎞�̏���
	static void wheel( GLFWwindow *window, double x, double y )
	{
		// ���̃C���X�^���X��this�|�C���^�𓾂�
		Window *const instance( static_cast< Window* >( glfwGetWindowUserPointer( window ) ) );

		if ( instance != NULL )
		{
			// ���[���h���W�n�ɑ΂���f�o�C�X���W�n�̊g�嗦���X�V����
			instance->scale += static_cast< GLfloat >( y );
		}
	}

	// �L�[�{�[�h���쎞�̏���
	static void keyboard( GLFWwindow *window, int key, int scancode, int action, int mods )
	{
		// ���̃C���X�^���X��this�|�C���^�𓾂�
		Window *const instance( static_cast< Window* >( glfwGetWindowUserPointer( window ) ) );

		if ( instance != NULL )
		{
			// �L�[�̏�Ԃ�ۑ�����
			instance->keyStatus = action;
		}
	}

	// ���邳�����o��
	void getBrightness( GLfloat *brightness )
	{
		brightness[ 0 ] = brightness[ 1 ] = brightness[ 2 ] = this->blightness * 0.1f;
		brightness[ 3 ] = 1.0f;
	}

};