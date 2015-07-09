/*
 * glsl_shader.h
 *
 *  Created on: Nov 27, 2011
 *      Author: Hordur Johannsson
 */

#ifndef GLSL_SHADER
#define GLSL_SHADER

#define GLM_FORCE_RADIANS

#include <GL/glew.h>
#include <memory>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "../utils/Conf.h"

namespace gllib
{
enum ShaderType { VERTEX = GL_VERTEX_SHADER,
                  FRAGMENT = GL_FRAGMENT_SHADER,
                  GEOMETRY = GL_GEOMETRY_SHADER,
                  TESS_CONTROL = GL_TESS_CONTROL_SHADER,
                  TESS_EVALUATION = GL_TESS_EVALUATION_SHADER };

/**
 * A GLSL shader program.
 */
 class Program
 {
     public:
        /**
         * Construct an empty shader program.
         */
        Program ();

        /**
         * Destruct the shader program.
         */
        ~Program ();

        /**
         * Add a new shader object to the program.
         */
        bool addShaderText (const std::string& text, ShaderType shader_type);

        /**
         * Add a new shader object to the program.
         */
        bool addShaderFile (const std::string& text, ShaderType shader_type);

        /**
         * Link the program.
         */
        bool link ();

        /**
         * Return true if the program is linked.
         */
        bool isLinked ();

        /**
         * Use the program.
         */
        void use ();

        // Set uniforms
        void setUniform (const std::string& name, const glm::mat4 & v);
        void setUniform (const std::string& name, const glm::vec3 & v);
        void printActiveUniforms ();
        void printActiveAttribs ();

        GLuint programId () { return program_id_; }

        static std::shared_ptr<gllib::Program> loadProgramFromFile (const std::string& vertex_shader_file, const std::string& fragment_shader_file);

     private:
        GLuint program_id_;
 };

 GLenum getGLError ();
 void printShaderInfoLog (GLuint shader);
 void printProgramInfoLog (GLuint program);

}

#endif
