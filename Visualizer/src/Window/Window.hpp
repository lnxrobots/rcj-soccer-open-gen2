#pragma once

#include <GLFW/glfw3.h>

#include <string>

struct WindowData
{
    std::string title;
    uint32_t width, height;
};

class Window
{
public:
    Window(const std::string& title, uint32_t width, uint32_t height);
    ~Window();

public:
    void Begin();
    void Update();
    bool ShouldClose();

    inline void* GetNativeWindow() { return m_Window; };
    inline uint32_t GetWidth() { return m_WindowData.width; }
    inline uint32_t GetHeight() { return m_WindowData.height; }
    
private:
    GLFWwindow* m_Window;
    WindowData m_WindowData;
    float m_Time = 0.0f;

private:
    bool InitGLFW();
    void PrepareImgui();
};
