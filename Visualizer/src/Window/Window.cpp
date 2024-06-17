#include "Window/Window.hpp"
#include "Window/ImGuiRenderer.hpp"

#include <iostream>

#include <glad/glad.h>
#include <imgui.h>

static bool s_GLFWInitialized = false;

static const char* ImGui_ImplGlfw_GetClipboardText(void* user_data)
{
    return glfwGetClipboardString((GLFWwindow*)user_data);
}

static void ImGui_ImplGlfw_SetClipboardText(void* user_data, const char* text)
{
    glfwSetClipboardString((GLFWwindow*)user_data, text);
}

Window::Window(const std::string& title, uint32_t width, uint32_t height)
{
    if (!s_GLFWInitialized)
    {
        if (!InitGLFW())
        {
            std::cout << "Failed to initialize GLFW!" << std::endl;
            return;
        }
        s_GLFWInitialized = true;
    }

    m_Window = glfwCreateWindow(width, height, title.c_str(), NULL, NULL);
    if (!m_Window)
    {
        std::cout << "Failed to create window!" << std::endl;
        glfwTerminate();
        return;
    }

    glfwMakeContextCurrent(m_Window);

    int tempWidth, tempHeight;
    glfwGetWindowSize(m_Window, &tempWidth, &tempHeight);

    m_WindowData.title = title;
    m_WindowData.width = tempWidth;
    m_WindowData.height = tempHeight;

    glfwSetWindowSizeLimits(m_Window, 640, 420, GLFW_DONT_CARE, GLFW_DONT_CARE);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize Glad!" << std::endl;
    }

    glfwSetWindowUserPointer(m_Window, &m_WindowData);

    //Window Resize Callback
    glfwSetWindowSizeCallback(m_Window, [](GLFWwindow* window, int width, int height)
    {
        WindowData& data = *(WindowData*)glfwGetWindowUserPointer(window);

        data.width = width;
        data.height = height;
    });

    PrepareImgui();
}

Window::~Window()
{
    glfwDestroyWindow(m_Window);
}

void Window::Begin()
{   
    // Clear Screen
    glClear(GL_COLOR_BUFFER_BIT);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

    // Prepare imgui
    ImGuiIO& io = ImGui::GetIO();
    io.DisplaySize = ImVec2((float)m_WindowData.width, (float)m_WindowData.height);

    float time = (float)glfwGetTime();
    io.DeltaTime = time > 0.0 ? (time - m_Time) : (1.0f / 60.0f);
    m_Time = time;

    ImGui_ImplOpenGL3_NewFrame();
    ImGui::NewFrame();
}

void Window::Update()
{
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(m_Window);
    glfwPollEvents();
}

bool Window::ShouldClose()
{
    return glfwWindowShouldClose(m_Window);
}

bool Window::InitGLFW()
{
    bool state = glfwInit();

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    #ifdef LNXVIS_PLATFORM_OSX
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    #endif

    //glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    
    glfwSwapInterval(1);
    
    return state;
}

void Window::PrepareImgui()
{
    ImGui::CreateContext();
    ImGui::StyleColorsDark();

    ImGuiIO& io = ImGui::GetIO();
    io.BackendFlags |= ImGuiBackendFlags_HasMouseCursors;
    io.BackendFlags |= ImGuiBackendFlags_HasSetMousePos;

    io.SetClipboardTextFn = ImGui_ImplGlfw_SetClipboardText; 
    io.GetClipboardTextFn = ImGui_ImplGlfw_GetClipboardText;

    //From ImGui main file
    io.KeyMap[ImGuiKey_Tab] = GLFW_KEY_TAB;
    io.KeyMap[ImGuiKey_LeftArrow] = GLFW_KEY_LEFT;
    io.KeyMap[ImGuiKey_RightArrow] = GLFW_KEY_RIGHT;
    io.KeyMap[ImGuiKey_UpArrow] = GLFW_KEY_UP;
    io.KeyMap[ImGuiKey_DownArrow] = GLFW_KEY_DOWN;
    io.KeyMap[ImGuiKey_PageUp] = GLFW_KEY_PAGE_UP;
    io.KeyMap[ImGuiKey_PageDown] = GLFW_KEY_PAGE_DOWN;
    io.KeyMap[ImGuiKey_Home] = GLFW_KEY_HOME;
    io.KeyMap[ImGuiKey_End] = GLFW_KEY_END;
    io.KeyMap[ImGuiKey_Insert] = GLFW_KEY_INSERT;
    io.KeyMap[ImGuiKey_Delete] = GLFW_KEY_DELETE;
    io.KeyMap[ImGuiKey_Backspace] = GLFW_KEY_BACKSPACE;
    io.KeyMap[ImGuiKey_Space] = GLFW_KEY_SPACE;
    io.KeyMap[ImGuiKey_Enter] = GLFW_KEY_ENTER;
    io.KeyMap[ImGuiKey_Escape] = GLFW_KEY_ESCAPE;
    io.KeyMap[ImGuiKey_KeyPadEnter] = GLFW_KEY_KP_ENTER;
    io.KeyMap[ImGuiKey_A] = GLFW_KEY_A;
    io.KeyMap[ImGuiKey_C] = GLFW_KEY_C;
    io.KeyMap[ImGuiKey_V] = GLFW_KEY_V;
    io.KeyMap[ImGuiKey_X] = GLFW_KEY_X;
    io.KeyMap[ImGuiKey_Y] = GLFW_KEY_Y;
    io.KeyMap[ImGuiKey_Z] = GLFW_KEY_Z;

    ImGui_ImplOpenGL3_Init("#version 330 core");

    glfwSetMouseButtonCallback(static_cast<GLFWwindow*>(m_Window), [](GLFWwindow* window, int button, int action, int mods)
    {
        ImGuiIO& io = ImGui::GetIO();

        switch (action)
        {
            case GLFW_PRESS:
            {
                io.MouseDown[button] = true;
                break;
            }
            case GLFW_RELEASE:
            {
                io.MouseDown[button] = false;
                break;
            }
        }
    });

    io.IniFilename = nullptr;

    glfwSetCursorPosCallback(static_cast<GLFWwindow*>(m_Window), [](GLFWwindow* window, double xOffset, double yOffset)
    {
        ImGuiIO& io = ImGui::GetIO();

        io.MousePos = ImVec2((float)xOffset, (float)yOffset);
    });

    glfwSetKeyCallback(static_cast<GLFWwindow*>(m_Window), [](GLFWwindow* window, int key, int scancode, int action, int mods)
    {
        ImGuiIO& io = ImGui::GetIO();

        switch (action)
        {
            case (bool)GLFW_PRESS || (bool)GLFW_REPEAT:
            {
                io.KeysDown[key] = true;

                io.KeyCtrl = io.KeysDown[GLFW_KEY_LEFT_CONTROL] || io.KeysDown[GLFW_KEY_RIGHT_CONTROL];
                io.KeyShift = io.KeysDown[GLFW_KEY_LEFT_SHIFT] || io.KeysDown[GLFW_KEY_RIGHT_SHIFT];
                io.KeyAlt = io.KeysDown[GLFW_KEY_LEFT_ALT] || io.KeysDown[GLFW_KEY_RIGHT_ALT];
                io.KeySuper = io.KeysDown[GLFW_KEY_LEFT_SUPER] || io.KeysDown[GLFW_KEY_RIGHT_SUPER];
                break;
            }
            case GLFW_RELEASE:
            {
                io.KeysDown[key] = false;
                break;
            }
        }
    });

    glfwSetCharCallback(static_cast<GLFWwindow*>(m_Window), [](GLFWwindow* window, unsigned int keycode)
    {
        ImGuiIO& io = ImGui::GetIO();

        if (keycode > 0 && keycode < 0x10000)
            io.AddInputCharacter((unsigned short)keycode);
    });
}
