#include "visualization/Shader.hpp"

#include <GL/glew.h>
#include <fstream>
#include <iostream>
#include <sstream>

namespace visualization
{

Shader::~Shader()
{
    if (m_program != 0)
    {
        glDeleteProgram(m_program);
    }
}

bool Shader::load(const std::string& vertexPath, const std::string& fragmentPath)
{
    const std::string vertexSource = loadSource(vertexPath);
    const std::string fragmentSource = loadSource(fragmentPath);

    if (vertexSource.empty() || fragmentSource.empty())
    {
        return false;
    }

    const GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    const GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);

    if (!compileShader(vertexShader, vertexSource) || !compileShader(fragmentShader, fragmentSource))
    {
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
        return false;
    }

    m_program = glCreateProgram();
    glAttachShader(m_program, vertexShader);
    glAttachShader(m_program, fragmentShader);
    glLinkProgram(m_program);

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    GLint linkStatus = GL_FALSE;
    glGetProgramiv(m_program, GL_LINK_STATUS, &linkStatus);
    if (linkStatus != GL_TRUE)
    {
        GLint logLength = 0;
        glGetProgramiv(m_program, GL_INFO_LOG_LENGTH, &logLength);
        std::string log(logLength, '\0');
        glGetProgramInfoLog(m_program, logLength, nullptr, log.data());
        std::cerr << "Shader link error: " << log << '\n';
        glDeleteProgram(m_program);
        m_program = 0;
        return false;
    }

    return true;
}

void Shader::use() const
{
    if (m_program != 0)
    {
        glUseProgram(m_program);
    }
}

GLuint Shader::id() const noexcept
{
    return m_program;
}

GLint Shader::uniformLocation(const std::string& name) const
{
    return glGetUniformLocation(m_program, name.c_str());
}

std::string Shader::loadSource(const std::string& path)
{
    std::ifstream file(path);
    if (!file)
    {
        std::cerr << "Unable to open shader file: " << path << '\n';
        return {};
    }

    std::ostringstream source;
    source << file.rdbuf();
    return source.str();
}

bool Shader::compileShader(GLuint shader, const std::string& source)
{
    const char* sourceCStr = source.c_str();
    glShaderSource(shader, 1, &sourceCStr, nullptr);
    glCompileShader(shader);

    GLint compileStatus = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compileStatus);
    if (compileStatus != GL_TRUE)
    {
        GLint logLength = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &logLength);
        std::string log(logLength, '\0');
        glGetShaderInfoLog(shader, logLength, nullptr, log.data());
        std::cerr << "Shader compile error: " << log << '\n';
        return false;
    }

    return true;
}

} // namespace visualization
