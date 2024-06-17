#pragma once

typedef unsigned int uint32_t;

class Panel
{
public:
    Panel(uint32_t x, uint32_t y, uint32_t width, uint32_t height)
        : m_X(x), m_Y(y), m_Width(width), m_Height(height)
    {}
    virtual ~Panel()
    {}

    void SetPosition(uint32_t x, uint32_t y)
    {
        m_X = x;
        m_Y = y;
    }

    void SetSize(uint32_t width, uint32_t height)
    {
        m_Width = width;
        m_Height = height;
    }

protected:
    uint32_t m_X;
    uint32_t m_Y;

    uint32_t m_Width;
    uint32_t m_Height;

public:
    virtual void OnRender() = 0;
};