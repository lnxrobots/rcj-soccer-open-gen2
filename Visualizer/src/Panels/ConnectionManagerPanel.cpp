#include "Panels/ConnectionManagerPanel.hpp"

#include <imgui.h>
#include <misc/cpp/imgui_stdlib.h>

ConnectionManagerPanel::ConnectionManagerPanel()
    : Panel(0, 0, 300, 200), m_Connection(), m_InputAdress(""), m_ConnectionStatus("")
{}

ConnectionManagerPanel::~ConnectionManagerPanel()
{}

void ConnectionManagerPanel::OnRender()
{
    ImGui::SetNextWindowPos(ImVec2(Panel::m_X, Panel::m_Y));
    ImGui::SetNextWindowSize(ImVec2(Panel::m_Width, Panel::m_Height));
    ImGui::Begin("Connection Manager", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

    bool enterPressed = ImGui::InputTextWithHint("Adress", "192.168.68.121:8765", &m_InputAdress, ImGuiInputTextFlags_EnterReturnsTrue); 

    if (m_Connection.GetConnectionStatus() != m_Connection.ConnectionStatus::CONNECTED)
    {
        if (ImGui::Button("Connect") || enterPressed)
            m_Connection.Connect("ws://" + m_InputAdress + "/");
    }
    else
    {
        if (ImGui::Button("Disconnect"))
            m_Connection.Disconnect();
    }

    if (m_Connection.GetConnectionStatus() == m_Connection.ConnectionStatus::CONNECTION_ERROR)
    {
        m_ConnectionStatus = "Failed to Connect: " + m_Connection.GetErrorMessage();
    }
    else if (m_Connection.GetConnectionStatus() != m_Connection.ConnectionStatus::CONNECTED)
    {
        m_ConnectionStatus = "Disconnected";
    }
    else
    {
        m_ConnectionStatus = "Successfully Connected";
    }

    ImGui::Text("Status: %s", m_ConnectionStatus.c_str());

    if (m_Connection.GetConnectionStatus() == m_Connection.ConnectionStatus::CONNECTED)
    {
        ImGui::Text("Server: %s", m_Connection.GetServer().c_str());
        ImGui::Text("Server Adress: %s", m_Connection.GetServerAdress().c_str());

        ImGui::NewLine();

        ImGui::Text("Last Packet Size: %i", m_Connection.GetBufferSize());
        ImGui::Text("Packets per Second: %i", m_Connection.GetMessagesPerSecond());
    }

    ImGui::End();
}
