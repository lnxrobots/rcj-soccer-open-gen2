#include "Panels/CalibrationPanel.hpp"

#include <imgui.h>

#include <algorithm>

ImVec4 RGBtoHSV(const ImVec4& RGBColor);

CalibrationPanel::CalibrationPanel(CameraImagePanel& cameraImage, ConnectionManagerPanel& connectionManager)
    : Panel(956, 0, 324, 400), m_CameraImage(cameraImage), m_ConnectionManager(connectionManager), m_CalibrationMethod(CalibrationMethod::POINT_SELECTION), m_Calibrating(false), m_SelectedColor(0.0f, 0.0f, 0.0f, 0.0f), m_HThreshold(1.0f), m_SThreshold(1.0f), m_VThreshold(1.0f)
{
    m_CurrentCalibrationData = &m_BallCalibrationData;
}

CalibrationPanel::~CalibrationPanel()
{}

void CalibrationPanel::OnRender()
{
    ImGui::SetNextWindowPos(ImVec2(Panel::m_X, Panel::m_Y));
    ImGui::SetNextWindowSize(ImVec2(Panel::m_Width, Panel::m_Height));
    ImGui::Begin("Camera Calibration", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

    /*
    ImGui::Text("Calibration Method");

    const char* calibrationMethods[] = { "Rect Selection", "Point Selection" };

    ImGui::Combo("", (int*)&m_CalibrationMethod, calibrationMethods, IM_ARRAYSIZE(calibrationMethods));
    */

    ImGui::Text("Calibration Target");
    const char* calibrationTargets[] = { "Ball", "Blue Goal", "Yellow Goal" };
    ImGui::Combo(" ", &m_CalibrationTarget, calibrationTargets, IM_ARRAYSIZE(calibrationTargets));

    if (m_CalibrationTarget == 0)
        m_CurrentCalibrationData = &m_BallCalibrationData;
    else if (m_CalibrationTarget == 1)
        m_CurrentCalibrationData = &m_BlueGoalCalibrationData;
    else if (m_CalibrationTarget == 2)
        m_CurrentCalibrationData = &m_YellowGoalCalibrationData;

    bool status = false;

    switch (m_CalibrationMethod)
    {
    case CalibrationPanel::NONE:
        break;

    case CalibrationPanel::RECT_SELECTION:
        m_CameraImage.EnableSelection();

        status = RenderRectCalibration();
        break;
    case CalibrationPanel::POINT_SELECTION:
        m_CameraImage.DisableSelection();

        status = RenderPointCalibration();

        break;

    default:
        break;
    }

    ImGui::Separator();

    ImGui::Text("Thresholds");
    ImGui::SliderFloat("H (%)", &m_HThreshold, 0.0f, 8.0f, "%.3f");
    ImGui::SliderFloat("S (%)", &m_SThreshold, 0.0f, 8.0f, "%.3f");
    ImGui::SliderFloat("V (%)", &m_VThreshold, 0.0f, 8.0f, "%.3f");

    ImVec4 minCalibrationColorHSVWithThresholds = m_CurrentCalibrationData->minCalibrationColorHSV;
    ImVec4 maxCalibrationColorHSVWithThresholds = m_CurrentCalibrationData->maxCalibrationColorHSV;

    minCalibrationColorHSVWithThresholds.x = (std::max)((float)(m_CurrentCalibrationData->minCalibrationColorHSV.x - 180.0f * (m_HThreshold / 100.0f)), 0.0f);
    minCalibrationColorHSVWithThresholds.y = (std::max)((float)(m_CurrentCalibrationData->minCalibrationColorHSV.y - 255.0f * (m_SThreshold / 100.0f)), 0.0f);
    minCalibrationColorHSVWithThresholds.z = (std::max)((float)(m_CurrentCalibrationData->minCalibrationColorHSV.z - 255.0f * (m_VThreshold / 100.0f)), 0.0f);

    maxCalibrationColorHSVWithThresholds.x = (std::min)((float)(m_CurrentCalibrationData->maxCalibrationColorHSV.x + 180.0f * (m_HThreshold / 100.0f)), 180.0f);
    maxCalibrationColorHSVWithThresholds.y = (std::min)((float)(m_CurrentCalibrationData->maxCalibrationColorHSV.y + 255.0f * (m_SThreshold / 100.0f)), 255.0f);
    maxCalibrationColorHSVWithThresholds.z = (std::min)((float)(m_CurrentCalibrationData->maxCalibrationColorHSV.z + 255.0f * (m_VThreshold / 100.0f)), 255.0f);

    // Lower boundery
    ImGui::ColorButton("Lower boundery", ToImGuiColorHSV(minCalibrationColorHSVWithThresholds), ImGuiColorEditFlags_InputHSV | ImGuiColorEditFlags_DisplayHSV);
    ImGui::SameLine();
    ImGui::Text("Lower: H: %i, S: %i, V: %i", (int)(minCalibrationColorHSVWithThresholds.x), (int)(minCalibrationColorHSVWithThresholds.y), (int)(minCalibrationColorHSVWithThresholds.z));

    ImGui::ColorButton("Upper boundery", ToImGuiColorHSV(maxCalibrationColorHSVWithThresholds), ImGuiColorEditFlags_InputHSV | ImGuiColorEditFlags_DisplayHSV);
    ImGui::SameLine();
    ImGui::Text("Upper: H: %i, S: %i, V: %i", (int)(maxCalibrationColorHSVWithThresholds.x), (int)(maxCalibrationColorHSVWithThresholds.y), (int)(maxCalibrationColorHSVWithThresholds.z));

    if (ImGui::Button("Clear Calibration"))
    {
        m_CurrentCalibrationData->minCalibrationColorHSV = ImVec4(180.0f, 225.0f, 255.0f, 255.0f);
        m_CurrentCalibrationData->maxCalibrationColorHSV = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);

        m_CurrentCalibrationData->minLessH = 181;
        m_CurrentCalibrationData->maxLessH = -1;

        m_CurrentCalibrationData->minMoreH = 181;
        m_CurrentCalibrationData->maxMoreH = -1;
    }

    if (ImGui::Button("Upload Calibration"))
    {
        m_CameraImage.ClearSelection();

        char calibrationData[6] = {
            (char)minCalibrationColorHSVWithThresholds.x, (char)minCalibrationColorHSVWithThresholds.y, (char)minCalibrationColorHSVWithThresholds.z,
            (char)maxCalibrationColorHSVWithThresholds.x, (char)maxCalibrationColorHSVWithThresholds.y, (char)maxCalibrationColorHSVWithThresholds.z 
            };

        m_ConnectionManager.GetConnection().SendMessage((char)(m_CalibrationTarget + 3), calibrationData, 6 * sizeof(char));
    }

    ImGui::End();
}

bool CalibrationPanel::RenderRectCalibration()
{
    if (m_CameraImage.IsSelection())
    {
        ImVec2 selectionPosition = m_CameraImage.GetSelectionPosition();
        ImVec2 selectionSize = m_CameraImage.GetSelectionSize();

        ImGui::Text("Calibration Selection position: %i, %i", (int)selectionPosition.x, (int)selectionPosition.y);
        ImGui::Text("Calibration Selection size: %i, %i", (int)selectionSize.x, (int)selectionSize.y);

        if (ImGui::Button("Use selection"))
        {
            m_CameraImage.GetCalibrationColors(m_CurrentCalibrationData->minCalibrationColorHSV, m_CurrentCalibrationData->maxCalibrationColorHSV);
        }
    }
    else
    {
        ImGui::Text("Select rectangle in the Camera Frame");
    }

    return true;
}

bool CalibrationPanel::RenderPointCalibration()
{
    if (m_CameraImage.IsSelection())
        m_CameraImage.ClearSelection();

    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
    {
        ImVec4 selectedColorRGB = m_CameraImage.GetSelectedColor();

        if (selectedColorRGB.x != -1.0f)
        {
            m_SelectedColor = RGBtoHSV(selectedColorRGB);

            UpdateColorBounds(m_SelectedColor, *m_CurrentCalibrationData);
        }
    }

    ImGui::ColorButton("Selected Color", ToImGuiColorHSV(m_SelectedColor), ImGuiColorEditFlags_InputHSV | ImGuiColorEditFlags_DisplayHSV);

    return true;
}

void CalibrationPanel::UpdateColorBounds(const ImVec4& newColor, CalibrationData& calibData)
{
    int hue = newColor.x;

    int minH = 0;
    int maxH = 0;

    if (hue < 90)
    {
        if (hue < calibData.minLessH)
        {
            calibData.minLessH = hue;
        }
        if (hue > calibData.maxLessH)
        {
            calibData.maxLessH = hue;
        }
    }
    else
    {
        if (hue < calibData.minMoreH)
        {
            calibData.minMoreH = hue;
        }
        if (hue > calibData.maxMoreH)
        {
            calibData.maxMoreH = hue;
        }
    }

    if (calibData.minLessH == 181 || calibData.maxLessH == -1)
    {
        minH = calibData.minMoreH;
        maxH = calibData.maxMoreH;

        m_TwoIntervals = false;
    }
    else if (calibData.minMoreH == 181 || calibData.maxMoreH == -1)
    {
        minH = calibData.minLessH;
        maxH = calibData.maxLessH;

        m_TwoIntervals = false;
    }
    else if ((180 - calibData.minMoreH) + calibData.maxLessH < calibData.maxMoreH - calibData.minLessH)
    {
        minH = calibData.minMoreH;
        maxH = calibData.maxLessH;

        m_TwoIntervals = true;
    }
    else
    {
        minH = calibData.minLessH;
        maxH = calibData.maxMoreH;

        m_TwoIntervals = false;
    }

    calibData.minCalibrationColorHSV.x = minH;
    calibData.maxCalibrationColorHSV.x = maxH;

    if (calibData.minCalibrationColorHSV.y > newColor.y)
        calibData.minCalibrationColorHSV.y = newColor.y;
    if (calibData.maxCalibrationColorHSV.y < newColor.y)
        calibData.maxCalibrationColorHSV.y = newColor.y;

    if (calibData.minCalibrationColorHSV.z > newColor.z)
        calibData.minCalibrationColorHSV.z = newColor.z;
    if (calibData.maxCalibrationColorHSV.z < newColor.z)
        calibData.maxCalibrationColorHSV.z = newColor.z;
}

ImVec4 CalibrationPanel::ToImGuiColorHSV(const ImVec4& HSVColor)
{
    return ImVec4(HSVColor.x / 180.0f, HSVColor.y / 255.0f, HSVColor.z / 255.0f, HSVColor.w / 255.0f);
}
