#include "Panels/UndercarriagePanel.hpp"

#include <imgui.h>
#include <misc/cpp/imgui_stdlib.h>

UndercarriagePanel::UndercarriagePanel(ConnectionManagerPanel& connectionManager)
    : Panel(956, 400, 324, 320), m_ConnectionManager(connectionManager), m_InputLineThreshold(0), m_LineThreshold(-1)
{
    for (uint32_t i = 0; i < 16; i++)
    {
        m_SensorValues[i] = -1;
    }

    for (uint32_t i = 0; i < 4; i++)
    {
        m_MotorValues[i] = -1;
    }

    for (uint32_t i = 0; i < 3; i++)
    {
        m_MotorsSpeedsCalibration[i] = 0;
    }
}

UndercarriagePanel::~UndercarriagePanel()
{}

void UndercarriagePanel::OnRender()
{
    ImGui::SetNextWindowPos(ImVec2(Panel::m_X, Panel::m_Y));
    ImGui::SetNextWindowSize(ImVec2(Panel::m_Width, Panel::m_Height));
    ImGui::Begin("Undercarriage", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

    ImGui::Text("Sensors (%i)", m_LineThreshold);

    ImGui::Text("Front: %i, %i, %i, %i", m_SensorValues[14], m_SensorValues[15], m_SensorValues[0], m_SensorValues[1]);
    ImGui::Text("Right: %i, %i, %i, %i", m_SensorValues[2], m_SensorValues[3], m_SensorValues[4], m_SensorValues[5]);
    ImGui::Text("Back: %i, %i, %i, %i", m_SensorValues[6], m_SensorValues[7], m_SensorValues[8], m_SensorValues[9]);
    ImGui::Text("Left: %i, %i, %i, %i", m_SensorValues[10], m_SensorValues[11], m_SensorValues[12], m_SensorValues[13]);

    if (m_InputLineThreshold < 0)
        m_InputLineThreshold = 0;
    else if (m_InputLineThreshold > 1000)
        m_InputLineThreshold = 1000;

    ImGui::InputInt("Line threshold", &m_InputLineThreshold, 50);

    if (ImGui::Button("Upload Line Calibration"))
    {
        m_ConnectionManager.GetConnection().SendMessage(6, &m_InputLineThreshold, sizeof(int));
    }

    ImGui::Separator();

    ImGui::Text("Motors");
    ImGui::Text("%i, %i, %i, %i", m_MotorValues[0], m_MotorValues[1], m_MotorValues[2], m_MotorValues[3]);

    for(uint32_t i = 0; i < 3; i++)
    {
        if (m_MotorsSpeedsCalibration[i] < 0)
            m_MotorsSpeedsCalibration[i] = 0;
        else if (m_MotorsSpeedsCalibration[i] > 255)
            m_MotorsSpeedsCalibration[i] = 255;
    }

    ImGui::InputInt("Normal speed", &m_MotorsSpeedsCalibration[0], 5, 10);
    ImGui::InputInt("Striking speed", &m_MotorsSpeedsCalibration[1], 5, 10);
    ImGui::InputInt("Rotating speed", &m_MotorsSpeedsCalibration[2], 5, 10);

    if (ImGui::Button("Upload Motors Calibration"))
    {
        m_ConnectionManager.GetConnection().SendMessage(7, &m_MotorsSpeedsCalibration, sizeof(int) * 3);
    }

    ImGui::End();
}

void UndercarriagePanel::UpdateSensors(int* sensorValues)
{
    for (uint32_t i = 0; i < 16; i++)
    {
        m_SensorValues[i] = sensorValues[i];
    }
}

void UndercarriagePanel::UpdateMotors(int* motorValues)
{
    for (uint32_t i = 0; i < 4; i++)
    {
        m_MotorValues[i] = motorValues[i];
    }
}
