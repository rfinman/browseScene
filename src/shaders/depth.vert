#version 330 core

layout (location = 0) in vec4 position;
layout (location = 1) in vec4 color;
layout (location = 2) in vec4 normal;
layout (location = 3) in vec4 object_id;

uniform mat4 t_inv;
uniform vec4 cam; //cx, cy, fx, fy
uniform float cols;
uniform float rows;
uniform float maxDepth;
uniform bool render_visualisation;

out vec4 vColor;
out vec4 vPosition;
out vec4 vNormRad;
out vec4 vObjectId;
out mat4 vT_Inv;

void main()
{
    vec4 vPosHome = t_inv * vec4(position.xyz, 1.0);
    if (vPosHome.z > maxDepth || vPosHome.z < 0.05) {
      vPosition = vec4(0.0,0.0,0.0,0.0);
    } else {
      vColor = color;
      vPosition = vec4(position.xyz, 1.0f);
      vNormRad =  normal;
      vObjectId = object_id;
      vT_Inv = t_inv;
    }
}


