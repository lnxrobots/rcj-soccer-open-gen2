#pragma once

#include "Panels/Panel.hpp"
#include "Connection.hpp"

#include <string>

class ConnectionManagerPanel : public Panel
{
public:
    ConnectionManagerPanel();
    ~ConnectionManagerPanel();

    Connection& GetConnection() { return m_Connection; }

public:
    virtual void OnRender() override;

private:
    Connection m_Connection;
    std::string m_InputAdress;
    std::string m_ConnectionStatus;
};
