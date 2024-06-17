#pragma once

#include "Panels/Panel.hpp"

#include <imgui.h>

class CameraImagePanel : public Panel
{
public:
    CameraImagePanel(uint32_t imageWidth, uint32_t imageHeight);
    ~CameraImagePanel();

public:
    virtual void OnRender() override;

    void LoadNewFrame(void* encodedFrameBuffer, uint32_t bufferSize);
    void AddBallBoundingBox(float x, float y, float w, float h);
    void AddGoalBoundingBox(float x, float y, float w, float h);
    void SetVerticalCrop01(float verticalCrop) { m_VerticalCrop01 = verticalCrop; }
    
    void Play();
    void Pause();

    void EnableSelection() { m_SelectionEnabled = true; }
    void DisableSelection() { m_SelectionEnabled = false; }
    void ClearSelection();
    bool IsSelection() const { return m_Selection.isSelected; }
    bool IsFrame() const { return m_FrameBuffer; }

    void SetImageScale(const ImVec2& scale) { m_ImageScale = scale; }

    ImVec2 GetSelectionPosition() const;
    ImVec2 GetSelectionSize() const;
    void GetCalibrationColors(ImVec4& outMinHSV, ImVec4& outMaxHSV) const;

    ImVec4 GetSelectedColor() { return CalculateSelectedColor(); }

private:
    struct Selection
    {
        ImVec2 start = { 0, 0 };
        ImVec2 end = { 0, 0 };
        bool isSelected = false;
        bool isSelecting = false;
    };

    struct BoundingBox
    {
        ImVec2 start = { -1, -1 };
        ImVec2 end = { -1, -1 };
    };

    uint32_t m_ImageWidth;
    uint32_t m_ImageHeight;

    ImVec2 m_ImageScale;

    BoundingBox m_BallBoundingBox;
    BoundingBox m_GoalBoundingBox;
    float m_VerticalCrop01;

    bool m_IsPaused;
    ImVec2 m_ImagePos;
    ImVec2 m_ImageMousePos;

    Selection m_Selection;
    bool m_SelectionEnabled;

    bool m_ShowBallBoundingBox;
    bool m_ShowGoalBoundingBox;
    bool m_ShowCropLine;

    unsigned char* m_FrameBuffer;
    uint32_t m_GPUTextureID;

private:
    ImVec4 CalculateSelectedColor();
    void UpdataSelelctionMouse();
    bool IsImageSelected();
};
