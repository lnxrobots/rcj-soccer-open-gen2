#pragma once

#include "Panels/Panel.hpp"
#include "Panels/ConnectionManagerPanel.hpp"

class UndercarriagePanel : public Panel
{
public:
    UndercarriagePanel(ConnectionManagerPanel& connectionManager);
    ~UndercarriagePanel();
public:
    virtual void OnRender() override;

    void UpdateSensors(int* sensorValues);
    void UpdateMotors(int* motorValues);
    void SetLineThreshold(int lineThreshold) {m_LineThreshold = lineThreshold; }
    void SetInputLineThreshold(int lineThreshold) { m_InputLineThreshold = lineThreshold; }

    void SetMotorsSpeedCalibration(int robotSpeed, int robotShootingSpeed, int robotRotatingSpeed) { m_MotorsSpeedsCalibration[0] = robotSpeed; m_MotorsSpeedsCalibration[1] = robotShootingSpeed; m_MotorsSpeedsCalibration[2] = robotRotatingSpeed; }

private:
    ConnectionManagerPanel& m_ConnectionManager;

    int m_SensorValues[16];
    int m_MotorValues[4];

    int m_InputLineThreshold;
    int m_LineThreshold;
    int m_MotorsSpeedsCalibration[3];
};