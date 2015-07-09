#version 330 core

in vec2 texcoord;
uniform float maxDepth;

out vec4 FragColor;

void main()
{
  if(dot(texcoord, texcoord) > 1.0) {
    discard;
  }
  float c = maxDepth * (2.0 * gl_FragCoord.z - 1.0);
  FragColor = vec4(c,c,c,1.0);
}
~  
