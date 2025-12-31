#version 330 core
layout(location = 0) in vec3 aPosition;
layout(location = 1) in float aIntensity;

uniform mat4 uViewProjection;
uniform float uPointSize;

out float vIntensity;

void main()
{
    vIntensity = aIntensity;
    gl_PointSize = uPointSize;
    gl_Position = uViewProjection * vec4(aPosition, 1.0);
}
