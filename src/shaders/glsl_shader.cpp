/*
 * glsl_shader.cpp
 *
 *  Created on: Nov 27, 2011
 *      Author: hordurj
 */

#include "glsl_shader.h"
#include <iostream>
#include <fstream>

using namespace gllib;

char * readTextFile(const char* filename)
{
    using namespace std;
    char* buf = NULL;
    ifstream file;
    file.open(filename, ios::in | ios::binary | ios::ate);

    if(file.is_open())
    {
        ifstream::pos_type size;
        size = file.tellg();
        buf = new char[size + static_cast<ifstream::pos_type>(1)];
        file.seekg(0, ios::beg);
        file.read(buf, size);
        file.close();
        buf[size] = 0;
    }
    return buf;
}

gllib::Program::Program()
{
    program_id_ = glCreateProgram();
}

gllib::Program::~Program()
{
}

void gllib::Program::setUniform(const std::string& name, const glm::mat4 & v)
{
    GLuint loc = glGetUniformLocation(program_id_, name.c_str());
    glUniformMatrix4fv(loc, 1, false, glm::value_ptr(v));
}

void gllib::Program::setUniform(const std::string& name, const glm::vec3 & v)
{
    GLuint loc = glGetUniformLocation(program_id_, name.c_str());
    glUniform3f(loc, v.x, v.y, v.z);
}

bool gllib::Program::addShaderText(const std::string& text, ShaderType shader_type)
{
    GLuint id;
    GLint compile_status;
    id = glCreateShader(shader_type);
    const char* source_list = text.c_str();

    glShaderSource(id, 1, &source_list, NULL);

    glCompileShader(id);
    printShaderInfoLog(id);
    glGetShaderiv(id, GL_COMPILE_STATUS, &compile_status);
    if(compile_status == GL_FALSE)
        return false;

    if(getGLError() != GL_NO_ERROR)
        return false;

    glAttachShader(program_id_, id);
    return true;
}

bool gllib::Program::addShaderFile(const std::string& filename, ShaderType shader_type)
{
    std::string fullPath = Conf::findShaderDir().append("/").append(filename);

    char* text = readTextFile(fullPath.c_str());

    if(text == NULL)
        return (false);

    std::string source(text);

    bool rval = addShaderText(text, shader_type);
    delete[] text;
    return rval;
}

bool gllib::Program::link()
{
    glLinkProgram(program_id_);
    printProgramInfoLog(program_id_);

    if(getGLError() != GL_NO_ERROR)
        return false;
    return true;
}

void gllib::Program::use()
{
    glUseProgram(program_id_);
}

GLenum gllib::getGLError()
{
    GLenum last_error = GL_NO_ERROR;
    GLenum error = glGetError();
    while(error != GL_NO_ERROR)
    {
        last_error = error;
        std::cout << "Error: OpenGL: "
                        << "gluErrorString(error) function mission"
                        << std::endl;
        error = glGetError();
    }
    return last_error;
}

void gllib::printShaderInfoLog(GLuint shader)
{
    GLsizei max_length;
    GLsizei length;
    GLchar* info_log;

    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
    max_length = length;
    info_log = new GLchar[length + 1];

    glGetShaderInfoLog(shader, max_length, &length, info_log);

    info_log[max_length] = 0;

    std::string log(info_log);

    if(log.length())
    {
        std::cout << "Shader info log: " << std::endl << info_log << std::endl;
    }

    delete[] info_log;
}

void gllib::printProgramInfoLog(GLuint program)
{
    GLsizei max_length;
    GLsizei length;
    GLchar* info_log;

    glGetProgramiv(program, GL_INFO_LOG_LENGTH, &length);
    max_length = length;
    info_log = new GLchar[length + 1];

    glGetProgramInfoLog(program, max_length, &length, info_log);

    info_log[max_length] = 0;

    std::string log(info_log);

    if(log.length())
    {
        std::cout << "Program info log: " << std::endl << info_log << std::endl;
    }

    delete[] info_log;
}

std::shared_ptr<gllib::Program> gllib::Program::loadProgramFromFile(const std::string& vertex_shader_file,
                                                                    const std::string& fragment_shader_file)
{
    // Load shader
    std::shared_ptr<Program> program = std::shared_ptr<Program>(new gllib::Program());
    if(!program->addShaderFile(vertex_shader_file, gllib::VERTEX))
    {
        std::cerr << "Failed loading vertex shader" << std::endl;
    }

    if(!program->addShaderFile(fragment_shader_file, gllib::FRAGMENT))
    {
        std::cerr << "Failed loading fragment shader" << std::endl;
    }

    program->link();

    return program;
}
