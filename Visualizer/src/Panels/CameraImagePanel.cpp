#include "Panels/CameraImagePanel.hpp"

#include "Utils.hpp"

#include <glad/glad.h>
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include <iostream>
#include <vector>
#include <cmath>

CameraImagePanel::CameraImagePanel(uint32_t imageWidth, uint32_t imageHeight)
    : Panel(300, 0, 656, 538), m_ImageWidth(imageWidth), m_ImageHeight(imageHeight), m_ImageScale(1.5f, 1.5f), m_VerticalCrop01(0.0f), m_IsPaused(false), m_ImagePos(0.0f, 0.0f), m_ImageMousePos(0.0f, 0.0f),
    m_SelectionEnabled(false), m_ShowBallBoundingBox(true), m_ShowGoalBoundingBox(true), m_ShowCropLine(false), m_FrameBuffer(nullptr), m_GPUTextureID(-1)
{
    glGenTextures(1, &m_GPUTextureID);
    glBindTexture(GL_TEXTURE_2D, m_GPUTextureID);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    // Create empty data for the buffer
    std::vector<GLubyte> emptyData(m_ImageWidth * m_ImageHeight * 4, 50);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, m_ImageWidth, m_ImageHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, &emptyData[0]);

    glBindTexture(GL_TEXTURE_2D, 0);
}

CameraImagePanel::~CameraImagePanel()
{
    if (m_FrameBuffer)
        stbi_image_free(m_FrameBuffer);
    
    glDeleteTextures(1, &m_GPUTextureID);
}

void CameraImagePanel::OnRender()
{
    if (IsFrame() && m_SelectionEnabled)
        UpdataSelelctionMouse();

    ImGui::SetNextWindowPos(ImVec2(Panel::m_X, Panel::m_Y));
    ImGui::SetNextWindowSize(ImVec2(Panel::m_Width, Panel::m_Height));
    ImGui::Begin("Camere Image", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

    // Upper Panel UI
    if (IsFrame())
    {
        if (!m_IsPaused)
        {
            if (ImGui::Button("Pause"))
                Pause();   
        }
        else
        {
            if (ImGui::Button("Play")) 
                Play();   
        }

        ImGui::SameLine();
        ImGui::Checkbox("Ball Bounding Box", &m_ShowBallBoundingBox);
        ImGui::SameLine();
        ImGui::Checkbox("Goal Bounding Box", &m_ShowGoalBoundingBox);
        ImGui::SameLine();
        ImGui::Checkbox("Crop Line", &m_ShowCropLine);
    }
    else
    {
        ImGui::Text("No frame input!");
    }

    // Image UI
    if (m_GPUTextureID != -1)
    {
        m_ImagePos = ImGui::GetCursorScreenPos();

        ImGui::Image((void*)(intptr_t)m_GPUTextureID, ImVec2((float)m_ImageWidth * m_ImageScale.x, (float)m_ImageHeight * m_ImageScale.y));
        ImDrawList* drawList = ImGui::GetWindowDrawList();

        if (m_BallBoundingBox.start.x != -1 && m_ShowBallBoundingBox)
        {
            drawList->AddRect(m_BallBoundingBox.start, m_BallBoundingBox.end, 0xFFFF0000, 0.0f, ImDrawFlags_None, 1.0f);
            drawList->AddText(ImVec2(m_BallBoundingBox.start.x, m_BallBoundingBox.start.y - 16), 0xFFFF0000, "Ball");
        }

        if (m_GoalBoundingBox.start.x != -1 && m_ShowGoalBoundingBox)
        {
            drawList->AddRect(m_GoalBoundingBox.start, m_GoalBoundingBox.end, 0xFF00FF00, 0.0f, ImDrawFlags_None, 1.0f);
            drawList->AddText(ImVec2(m_GoalBoundingBox.start.x, m_GoalBoundingBox.start.y - 16), 0xFF00FF00, "Goal");
        }

        if (m_ShowCropLine)
        {
            drawList->AddLine(ImVec2(m_ImagePos.x, m_ImagePos.y + m_ImageHeight * m_ImageScale.y * m_VerticalCrop01), ImVec2(m_ImagePos.x + m_ImageWidth * m_ImageScale.x, m_ImagePos.y + m_ImageHeight * m_ImageScale.y * m_VerticalCrop01), 0xFF0000FF);
        }

        if (m_Selection.isSelected || m_Selection.isSelecting)
        {
            ImVec2 end = m_Selection.isSelecting ? ImGui::GetMousePos() : m_Selection.end;
            drawList->AddRect(m_Selection.start, end, 0xFF00FF00, 0.0f, ImDrawFlags_None, 1.0f);
        }
    }

    ImGui::End();

    ImVec2 mousePos = ImGui::GetMousePos();
    m_ImageMousePos.x = mousePos.x - m_ImagePos.x;
    m_ImageMousePos.y = mousePos.y - m_ImagePos.y;
}

void CameraImagePanel::LoadNewFrame(void* encodedFrameBuffer, uint32_t bufferSize)
{
    if (m_IsPaused)
        return;

    // Decode the image
    int x, y, comp;

    unsigned char* loadedFrame;
    loadedFrame = stbi_load_from_memory((unsigned char*)encodedFrameBuffer, bufferSize, &x, &y, &comp, STBI_rgb_alpha);

    if (!loadedFrame)
    {
        std::cout << "Error decoding the frame: " << stbi_failure_reason() << std::endl;
        return;
    }

    stbi_image_free(m_FrameBuffer);
    m_FrameBuffer = loadedFrame;

    if (x != m_ImageWidth || y != m_ImageHeight)
    {
        std::cout << "Error loading the frame to ImagePanel, dynamic change of image size is not available yet" << std::endl;
        return;
    }

    // Upload to the GPU
    glBindTexture(GL_TEXTURE_2D, m_GPUTextureID);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, m_ImageWidth, m_ImageHeight, GL_RGBA, GL_UNSIGNED_BYTE, m_FrameBuffer);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void CameraImagePanel::AddBallBoundingBox(float x, float y, float w, float h)
{
    if (m_IsPaused)
        return;

    x *= m_ImageScale.x;
    y *= m_ImageScale.y;
    w *= m_ImageScale.x;
    h *= m_ImageScale.y;

    ImVec2 start = { (x - (w / 2.0f)) * (float)m_ImageWidth + (float) m_ImagePos.x, (y - (h / 2.0f)) * (float)m_ImageHeight + (float) m_ImagePos.y };
    ImVec2 end = { start.x + w * m_ImageWidth, start.y + h * m_ImageHeight };

    m_BallBoundingBox.start = start;
    m_BallBoundingBox.end = end;
}

void CameraImagePanel::AddGoalBoundingBox(float x, float y, float w, float h)
{
    if (m_IsPaused)
        return;

    x *= m_ImageScale.x;
    y *= m_ImageScale.y;
    w *= m_ImageScale.x;
    h *= m_ImageScale.y;

    ImVec2 start = { (x - (w / 2.0f)) * (float)m_ImageWidth + (float) m_ImagePos.x, (y - (h / 2.0f)) * (float)m_ImageHeight + (float) m_ImagePos.y };
    ImVec2 end = { start.x + w * m_ImageWidth, start.y + h * m_ImageHeight };

    m_GoalBoundingBox.start = start;
    m_GoalBoundingBox.end = end;
}

void CameraImagePanel::Play()
{
    m_IsPaused = false;
}

void CameraImagePanel::Pause()
{
    m_IsPaused = true;
}

void CameraImagePanel::ClearSelection()
{
    m_Selection.start = {0, 0};
    m_Selection.end = {0, 0};
    m_Selection.isSelecting = false;
    m_Selection.isSelected = false;
}

ImVec2 CameraImagePanel::GetSelectionPosition() const
{
    return ImVec2(m_Selection.start.x - m_ImagePos.x, m_Selection.start.y - m_ImagePos.y);
}

ImVec2 CameraImagePanel::GetSelectionSize() const
{
    return ImVec2(m_Selection.end.x - m_Selection.start.x, m_Selection.end.y - m_Selection.start.y);
}

void CameraImagePanel::GetCalibrationColors(ImVec4& outMinHSV, ImVec4& outMaxHSV) const
{
    if (m_FrameBuffer && m_Selection.isSelected)
    {
        float outMinH = 180.0f;
        float outMinS = 255.0f;
        float outMinV = 255.0f;

        float outMaxH = 0.0f;
        float outMaxS = 0.0f;
        float outMaxV = 0.0f;

        ImVec2 selectionSize = GetSelectionSize();

        int nSelectedPixels = (int)(selectionSize.x * selectionSize.y);

        unsigned char* selectedPart = new unsigned char[nSelectedPixels * 4];

        for (uint32_t i = 0; i < selectionSize.y; i++)
        {
            for (uint32_t j = 0; j < selectionSize.x; j++)
            {
                ImVec2 startPoint = GetSelectionPosition();

                uint32_t sourcePixel = (m_ImageWidth * (uint32_t)(startPoint.y + i) + (uint32_t)startPoint.x + j) * 4;
                uint32_t targetPixel = (i * (uint32_t)selectionSize.x + j) * 4;

            float r = m_FrameBuffer[sourcePixel + 0];
            float g = m_FrameBuffer[sourcePixel + 1];
            float b = m_FrameBuffer[sourcePixel + 2];

            ImVec4 HSVColor = RGBtoHSV(ImVec4(r / 255.0f, g / 255.0f, b / 255.0f, 1.0f));

            float h = HSVColor.x;
            float s = HSVColor.y;
            float v = HSVColor.z;

            if (h < outMinH)
                outMinH = h;
            if (s < outMinS)
                outMinS = s;
            if (v < outMinV)
                outMinV = v;

            if (h > outMaxH)
                outMaxH = h;
            if (s > outMaxS)
                outMaxS = s;
            if (v > outMaxV)
                outMaxV = v;

            selectedPart[targetPixel + 0] = (unsigned char)r;
            selectedPart[targetPixel + 1] = (unsigned char)g;
            selectedPart[targetPixel + 2] = (unsigned char)b;
            selectedPart[targetPixel + 3] = 255;
            }
        }

        delete[] selectedPart;

        outMinHSV = ImVec4(outMinH, outMinS, outMinV, 255.0f);
        outMaxHSV = ImVec4(outMaxH, outMaxS, outMaxV, 255.0f);
    }
}

ImVec4 CameraImagePanel::CalculateSelectedColor()
{
    if (!m_FrameBuffer)
        return ImVec4(-1.0f, -1.0f, -1.0f, -1.0f);

    if (!IsImageSelected())
        return ImVec4(-1.0f, -1.0f, -1.0f, -1.0f);

    uint32_t pixelPos = (((uint32_t)(m_ImageMousePos.x / m_ImageScale.x) + (uint32_t)(m_ImageMousePos.y / m_ImageScale.y) * m_ImageWidth) * 4);
    ImVec4 color((float)m_FrameBuffer[pixelPos + 0] / 255.0f, (float)m_FrameBuffer[pixelPos + 1] / 255.0f, (float)m_FrameBuffer[pixelPos + 2] / 255.0f, 1);

    return color;
}

void CameraImagePanel::UpdataSelelctionMouse()
{
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
    {
        if (IsImageSelected())
        {
            m_Selection.start = ImGui::GetMousePos();
            m_Selection.end = {0, 0};
            m_Selection.isSelecting = true;
            m_Selection.isSelected = false;
        }
    }

    if (ImGui::IsMouseReleased(ImGuiMouseButton_Left))
    {   
        if (m_Selection.isSelecting)
        {
            if (IsImageSelected())
            {
                m_Selection.end = ImGui::GetMousePos();

                if (m_Selection.end.x < m_Selection.start.x)
                {
                    m_Selection.start.x += m_Selection.end.x;
                    m_Selection.end.x = m_Selection.start.x - m_Selection.end.x;
                    m_Selection.start.x -= m_Selection.end.x;
                }

                if (m_Selection.end.y < m_Selection.start.y)
                {
                    m_Selection.start.y += m_Selection.end.y;
                    m_Selection.end.y = m_Selection.start.y - m_Selection.end.y;
                    m_Selection.start.y -= m_Selection.end.y;
                }

                m_Selection.isSelecting = false;

                // Check if selection is not too small
                if ((m_Selection.start.x == m_Selection.end.x) || (m_Selection.start.y == m_Selection.end.y))
                    m_Selection.isSelected = false;
                else
                    m_Selection.isSelected = true;
            }
            else
            {
                m_Selection.start = {0, 0};
                m_Selection.end = {0, 0};
                m_Selection.isSelecting = false;
                m_Selection.isSelected = false;   
            }
        }
    }
}

bool CameraImagePanel::IsImageSelected()
{
    return !(m_ImageMousePos.x / m_ImageScale.x > m_ImageWidth || m_ImageMousePos.y / m_ImageScale.y > m_ImageHeight || m_ImageMousePos.x / m_ImageScale.x < 0 || m_ImageMousePos.y / m_ImageScale.y < 0);
}
