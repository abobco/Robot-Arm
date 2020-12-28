attribute vec4 vPosition;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
	// vec4 aPos = vec4(vPosition, 1.0f);
	gl_Position = projection * view * model * vPosition;
}