#include "Panels/SystemInformationPanel.hpp"

#include <imgui.h>

SystemInformationPanel::SystemInformationPanel()
    : Panel(0, 200, 300, 338), m_CPUTemperature(0)
{
    for (uint32_t i = 0; i < 9; i++)
    {
        m_PositionHeatmap[i] = 0;
    }
}

SystemInformationPanel::~SystemInformationPanel()
{}

void SystemInformationPanel::OnRender()
{
    ImGui::SetNextWindowPos(ImVec2(Panel::m_X, Panel::m_Y));
    ImGui::SetNextWindowSize(ImVec2(Panel::m_Width, Panel::m_Height));
    ImGui::Begin("Raspberry System Information", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

    ImGui::Text("CPU temperature: %i°C", m_CPUTemperature);

    ImGui::Separator();

    ImGui::Text("Position Heatmap");
        
    for (uint32_t i = 0; i < 3; i++)
    {
        for (uint32_t j = 0; j < 3; j++)
        {
            //ImVec4 textColor(0, 0, 0, 255);
            //ImGui::ColorConvertHSVtoRGB(255.0f / 3.0f, 255, 255, textColor.x, textColor.y, textColor.z);

            ImVec4 textColor = ImColor::HSV((1.0f - m_PositionHeatmap[i * 3 + j]) / 3.0f, 1.0f, 1.0f);

            ImGui::PushStyleColor(ImGuiCol_Text, textColor);
            ImGui::Text("%f", m_PositionHeatmap[i * 3 + j]);
            ImGui::PopStyleColor();

            if (j != 2)
                ImGui::SameLine();
        }
    }


    ImGui::End();
}

void SystemInformationPanel::UpdateSystemInformation(uint32_t CPUTemperature)
{
    m_CPUTemperature = CPUTemperature;
}

void SystemInformationPanel::UpdatePositionHeatmap(float* heatmap)
{
    for (uint32_t i = 0; i < 9; i++)
    {
        m_PositionHeatmap[i] = heatmap[i];
    }
}
