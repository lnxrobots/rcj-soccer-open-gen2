#include "Connection.hpp"

#include <imgui.h>

#include <iostream>
#include <algorithm>

Connection::Connection()
    : m_ConnectionStatus(ConnectionStatus::DISCONNECTED), m_Server(""), m_ServerAdress(""), m_BufferSize(0), m_IsNewData(false), m_ErrorMessage(""),
      m_NewMessages(0), m_LastTime(ImGui::GetTime()), m_MessagesPerSecond(0)
{
    m_Endpoint.clear_access_channels(websocketpp::log::alevel::all);
    m_Endpoint.clear_error_channels(websocketpp::log::elevel::all);

    m_Endpoint.init_asio();
    m_Endpoint.start_perpetual();

    m_ConnectionThread.reset(new std::thread(&asioClient::run, &m_Endpoint));
}

Connection::~Connection()
{
    m_Endpoint.stop_perpetual();
    
    if (m_ConnectionStatus == ConnectionStatus::CONNECTED)
    
    {
        websocketpp::lib::error_code ec;
        m_Endpoint.close(m_ConnectionHandle, websocketpp::close::status::going_away, "", ec);

        if (ec)
        {
            std::cout << "> Error initiating close: " << ec.message() << std::endl;
        }
    }

   m_ConnectionThread->join();
}

bool Connection::Connect(const std::string& adress)
{
    m_ServerAdress = adress;

    if (m_ConnectionStatus == ConnectionStatus::CONNECTED)
    {
        std::cout << "Already connected" << std::endl;
        return false;
    }

    m_ConnectionStatus = ConnectionStatus::CONNECTING;

    websocketpp::lib::error_code ec;

    asioClient::connection_ptr nativeConnection = m_Endpoint.get_connection(adress, ec);

    if (ec)
    {
        m_ConnectionStatus = ConnectionStatus::CONNECTION_ERROR;
        m_ErrorMessage = ec.message();

        return false;
    }

    m_ConnectionHandle = nativeConnection->get_handle();

    nativeConnection->set_open_handler(websocketpp::lib::bind(
        &Connection::OnOpenCallback,
        this,
        &m_Endpoint,
        websocketpp::lib::placeholders::_1
    ));

    nativeConnection->set_close_handler(websocketpp::lib::bind(
        &Connection::OnCloseCallback,
        this,
        &m_Endpoint,
        websocketpp::lib::placeholders::_1
    ));

    nativeConnection->set_fail_handler(websocketpp::lib::bind(
        &Connection::OnErrorCallback,
        this,
        &m_Endpoint,
        websocketpp::lib::placeholders::_1
    ));

    nativeConnection->set_message_handler(websocketpp::lib::bind(
            &Connection::OnMessageCallback,
            this,
            websocketpp::lib::placeholders::_1,
            websocketpp::lib::placeholders::_2
        ));

    m_Endpoint.connect(nativeConnection);

    return true;
}

void Connection::Disconnect()
{
    if (m_ConnectionStatus != ConnectionStatus::CONNECTED)
        return;

    websocketpp::lib::error_code ec;

    m_Endpoint.close(m_ConnectionHandle, websocketpp::close::status::normal, "", ec);

    if (ec)
    {
        std::cout << "> Error initiating close: " << ec.message() << std::endl;
    }
}

void Connection::SendMessage(char messageType, void* data, uint32_t lenght)
{
    websocketpp::lib::error_code ec;

    char message[128];
    message[0] = messageType;
    *((int*)(message + 1)) = 5 + lenght;
    memcpy(message + 5, data, lenght);

    m_Endpoint.send(m_ConnectionHandle, message, lenght + 5, websocketpp::frame::opcode::binary, ec);

    if (ec)
        std::cout << "Error sending message: " << ec.message() << std::endl;
}

const char* Connection::GetMessageBuffer()
{
    m_IsNewData = false;
    return m_MessageBuffer.get(); 
}

void Connection::OnOpenCallback(asioClient* client, websocketpp::connection_hdl hdl)
{
    m_ConnectionStatus = ConnectionStatus::CONNECTED;


    asioClient::connection_ptr connection = client->get_con_from_hdl(hdl);
    m_Server = connection->get_response_header("Server");
}

void Connection::OnCloseCallback(asioClient* client, websocketpp::connection_hdl hdl)
{
    m_ConnectionStatus = ConnectionStatus::DISCONNECTED;
}

void Connection::OnErrorCallback(asioClient* client, websocketpp::connection_hdl hdl)
{
    asioClient::connection_ptr nativeConnection = client->get_con_from_hdl(hdl);

    m_ConnectionStatus = ConnectionStatus::CONNECTION_ERROR;
    m_ErrorMessage = nativeConnection->get_ec().message();
}

void Connection::OnMessageCallback(websocketpp::connection_hdl connectionHandle, asioClient::message_ptr message)
{
    if (ImGui::GetTime() - m_LastTime >= 1.0f)
    {
        m_LastTime = ImGui::GetTime();
        
        m_MessagesPerSecond = m_NewMessages;
        m_NewMessages = 0;
    }

    m_NewMessages++;

    m_BufferSize = (uint32_t)message->get_payload().size();

    if (m_BufferSize > m_MaxBufferSize)
    {
        std::cout << "Cannot copy received message: message too big (" << m_BufferSize << " > " << m_MaxBufferSize << ")" << std::endl;
        return;
    }

    m_MessageBufferMutex.lock();
    std::copy(message->get_payload().data(), message->get_payload().data() + m_BufferSize, m_MessageBuffer.get());
    m_MessageBufferMutex.unlock();
    
    m_IsNewData = true;
}
