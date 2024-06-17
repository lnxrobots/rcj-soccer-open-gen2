#pragma once

#include "Panels/Panel.hpp"
#include "Panels/CameraImagePanel.hpp"
#include "Panels/ConnectionManagerPanel.hpp"

class CalibrationPanel : public Panel
{
public:
    CalibrationPanel(CameraImagePanel& cameraImage, ConnectionManagerPanel& connectionManager);
    ~CalibrationPanel();

public:
    virtual void OnRender() override;

private:
    CameraImagePanel& m_CameraImage;
    ConnectionManagerPanel& m_ConnectionManager;

    enum CalibrationMethod
    {
        NONE = -1,
        RECT_SELECTION, POINT_SELECTION
    };

    struct CalibrationData
    {   
        ImVec4 minCalibrationColorHSV = { 180.0f, 225.0f, 255.0f, 255.0f };
        ImVec4 maxCalibrationColorHSV = { 0.0f, 0.0f, 0.0f, 0.0f };

        int minLessH = 181;
        int maxLessH = -1;

        int minMoreH = 181;
        int maxMoreH = -1;    
    };


    int m_CalibrationTarget = 0;

    CalibrationMethod m_CalibrationMethod;
    bool m_Calibrating;

    ImVec4 m_SelectedColor;

    CalibrationData* m_CurrentCalibrationData = nullptr;

    CalibrationData m_BallCalibrationData;
    CalibrationData m_BlueGoalCalibrationData;
    CalibrationData m_YellowGoalCalibrationData;

    /*
    ImVec4 m_MinCalibrationColorHSV = { 180.0f, 225.0f, 255.0f, 255.0f };
    ImVec4 m_MaxCalibrationColorHSV = { 0.0f, 0.0f, 0.0f, 0.0f };

    int m_MinLessH = 181;
    int m_MaxLessH = -1;

    int m_MinMoreH = 181;
    int m_MaxMoreH = -1;
    */

    bool m_TwoIntervals = false;

    float m_HThreshold, m_SThreshold, m_VThreshold;

private:
    bool RenderRectCalibration();
    bool RenderPointCalibration();
    
    void UpdateColorBounds(const ImVec4& newColor, CalibrationData& calibData);
    ImVec4 ToImGuiColorHSV(const ImVec4& HSVColor);
};
