#pragma once
// Connection through websockets wrapper of websocketpp lib

#pragma warning(push, 0)
#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>
#pragma warning(pop)

#include <string>
#include <memory>
#include <thread>
#include <mutex>

typedef websocketpp::client<websocketpp::config::asio_client> asioClient;

class Connection
{
public:
    Connection();
    virtual ~Connection();

public:
    enum ConnectionStatus
    {
        NONE = -1,
        DISCONNECTED, CONNECTED, CONNECTING, CONNECTION_ERROR
    };

public:
    bool Connect(const std::string& adress);
    void Disconnect();

    void SendMessage(char messageType, void* data, uint32_t lenght);

    ConnectionStatus GetConnectionStatus() const { return m_ConnectionStatus; }
    const std::string& GetServer() const { return m_Server; }
    const std::string& GetServerAdress() const { return m_ServerAdress; }

    bool IsNewData() const { return m_IsNewData; }

    const char* GetMessageBuffer();
    uint32_t GetBufferSize() const { return m_BufferSize; }

    // TODO: Mutexes should be entirely internal thing of Connection class
    std::mutex& GetMessageBufferMutex() { return m_MessageBufferMutex; }

    const std::string& GetErrorMessage() const { return m_ErrorMessage; }
    uint32_t GetMessagesPerSecond() const { return m_MessagesPerSecond; }

private:
    void OnOpenCallback(asioClient* client, websocketpp::connection_hdl hdl);
    void OnCloseCallback(asioClient* client, websocketpp::connection_hdl hdl);
    void OnErrorCallback(asioClient* client, websocketpp::connection_hdl hdl);
    
    void OnMessageCallback(websocketpp::connection_hdl connectionHandle, asioClient::message_ptr message);

private:
    ConnectionStatus m_ConnectionStatus;

    std::string m_Server;
    std::string m_ServerAdress;
    asioClient m_Endpoint;
    
    std::shared_ptr<std::thread> m_ConnectionThread;
    websocketpp::connection_hdl m_ConnectionHandle;

    static const uint32_t m_MaxBufferSize = 128 * 1024;
    std::unique_ptr<char[]> m_MessageBuffer = std::make_unique<char[]>(m_MaxBufferSize);
    std::mutex m_MessageBufferMutex;

    uint32_t m_BufferSize;
    bool m_IsNewData;

    std::string m_ErrorMessage;

    uint32_t m_NewMessages;
    double m_LastTime;
    uint32_t m_MessagesPerSecond;
};
