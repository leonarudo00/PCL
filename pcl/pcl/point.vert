#version 150 core
uniform vec2 size;
uniform float scale;
uniform vec2 location;
uniform mat4 projectionMatrix;
in vec4 position;

void main()
{
	gl_Position = projectionMatrix * position * vec4(2.0 * scale / size, 1.0, 1.0) + vec4(location, 0.0, 0.0);
}