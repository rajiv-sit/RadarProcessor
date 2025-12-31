#version 330 core
in float vIntensity;
out vec4 FragColor;

uniform float uIntensityScale;
uniform vec3 uBaseColor;

void main()
{
    float alpha = clamp(vIntensity * uIntensityScale, 0.0, 1.0);
    FragColor = vec4(uBaseColor * alpha, alpha);
}
