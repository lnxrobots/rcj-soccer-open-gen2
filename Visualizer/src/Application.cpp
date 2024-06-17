#include "Window/Window.hpp"
#include "Panels/CameraImagePanel.hpp"
#include "Panels/ConnectionManagerPanel.hpp"
#include "Panels/SystemInformationPanel.hpp"
#include "Panels/CalibrationPanel.hpp"
#include "Panels/UndercarriagePanel.hpp"

#include <GLFW/glfw3.h>

#if defined LNXVIS_PLATFORM_WIN && defined LNXVIS_RELEASE
    #pragma comment(linker, "/SUBSYSTEM:windows /ENTRY:mainCRTStartup")
#endif

int main()
{
    // Create window, including imgui
    Window window("LNX Visualizer", 1280, 720); //1280 x 720

    uint32_t windowHeight = window.GetHeight();
    uint32_t windowWidth = window.GetWidth();

    // Initialize all the panels
    ConnectionManagerPanel connectionManager;
    CameraImagePanel cameraImage(640, 480);
    SystemInformationPanel systemInformation;
    CalibrationPanel calibration(cameraImage, connectionManager);
    UndercarriagePanel undercarriage(connectionManager);

    double currentTime = glfwGetTime();
    double lastTime = currentTime;

    // Window loop
    while (!window.ShouldClose())
    {
        currentTime = glfwGetTime();

        if (currentTime - lastTime >= 1.0 / 60.0f)
        {
            lastTime = currentTime;

            window.Begin();

            uint32_t windowHeight = window.GetHeight();
            uint32_t windowWidth = window.GetWidth();

            uint32_t leftPanelsWidth = 280;
            uint32_t rightPanelsWidth = 330;

            connectionManager.SetPosition(0, 0);
            connectionManager.SetSize(leftPanelsWidth, 200);

            cameraImage.SetPosition(leftPanelsWidth, 0);
            cameraImage.SetSize(windowWidth - leftPanelsWidth - rightPanelsWidth, (windowWidth - leftPanelsWidth - rightPanelsWidth) * (480.0f / 640.0f) + 50);
            
            float scale = (float)(windowWidth - leftPanelsWidth - rightPanelsWidth - 16) / 640.0f;

            cameraImage.SetImageScale(ImVec2(scale, scale));

            systemInformation.SetPosition(0, 200);
            systemInformation.SetSize(leftPanelsWidth, 338);

            calibration.SetPosition(windowWidth - rightPanelsWidth, 0);
            calibration.SetSize(rightPanelsWidth, 300);

            undercarriage.SetPosition(windowWidth - rightPanelsWidth, 300);
            undercarriage.SetSize(rightPanelsWidth, 438);

            // Check for new messages in websocket
            if (connectionManager.GetConnection().IsNewData() && connectionManager.GetConnection().GetBufferSize() > 5)
            {
                const char* buffer = connectionManager.GetConnection().GetMessageBuffer();
                connectionManager.GetConnection().GetMessageBufferMutex().lock();

                int packetID = *(char*)buffer;
                int packetSize = *(int*)(buffer + 1);

                
                if (packetID == 0)
                {
                    undercarriage.SetInputLineThreshold(*(int*)(buffer + 5));
                    undercarriage.SetLineThreshold(*(int*)(buffer + 5));
                    undercarriage.SetMotorsSpeedCalibration(*(int*)(buffer + 9), *(int*)(buffer + 13), *(int*)(buffer + 17));

                    cameraImage.SetVerticalCrop01(*(float*)(buffer + 21));
                }
                
                
                else if (packetID == 2)
                {
                    systemInformation.UpdateSystemInformation(*(int*)(buffer + 5));

                    undercarriage.UpdateSensors((int*)(buffer + 21));
                    undercarriage.UpdateMotors((int*)(buffer + 85));

                        
                    systemInformation.UpdatePositionHeatmap((float*)(buffer + 105));

                    int frameSize = *(int*)(buffer + 173);

                    if (frameSize != 0)
                        cameraImage.LoadNewFrame((unsigned char*)(buffer + 177), frameSize);

                    float ballX = *(float*)(buffer + 141);
                    float ballY = *(float*)(buffer + 145);

                    float ballWidth = *(float*)(buffer + 149);
                    float ballHeight = *(float*)(buffer + 153);

                    cameraImage.AddBallBoundingBox(ballX, ballY, ballWidth, ballHeight);

                    float goalX = *(float*)(buffer + 157);
                    float goalY = *(float*)(buffer + 161);

                    float goalWidth = *(float*)(buffer + 165);
                    float goalHeight = *(float*)(buffer + 169);

                    cameraImage.AddGoalBoundingBox(goalX, goalY, goalWidth, goalHeight);
                }

                else if (packetID == 6)
                {
                    undercarriage.SetLineThreshold(*(int*)(buffer + 5));
                }

                connectionManager.GetConnection().GetMessageBufferMutex().unlock();

                //std::cout << packetID << " " << packetSize << " " << connectionManager.GetConnection().GetBufferSize() << std::endl;

                // Update data in the panels    
                //uint32_t CPUTemperature = *(int*)buffer;
            }

            // Render all the panels
            connectionManager.OnRender();
            cameraImage.OnRender();
            systemInformation.OnRender();
            calibration.OnRender();
            undercarriage.OnRender();

            /*
            ImGui::SetNextWindowPos(ImVec2(0, 538));
            ImGui::SetNextWindowSize(ImVec2(windowWidth / 5, windowHeight ));
            ImGui::ShowMetricsWindow();
            */

            window.Update();
        }
    }
}
