#pragma once

#include "Panels/Panel.hpp"

class SystemInformationPanel : public Panel
{
public:
    SystemInformationPanel();
    ~SystemInformationPanel();
public:
    virtual void OnRender() override;

    void UpdateSystemInformation(uint32_t CPUTemperature);
    void UpdatePositionHeatmap(float* heatmap);

private:
    uint32_t m_CPUTemperature;
    float m_PositionHeatmap[9];
};