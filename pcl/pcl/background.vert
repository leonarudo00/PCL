#version 150 core
 
// �e�N�X�`�����W
out vec2 texcoord;
 
void main()
{
  // �e�N�X�`�����W�����߂�
  texcoord = vec2(gl_VertexID % 2, gl_VertexID / 2);
  
  // �e�N�X�`�����W���璸�_���W�����߂ďo��
  gl_Position = vec4(texcoord * 2.0 - 1.0, 0.0, 1.0);
}