#include "mjpeg_server.h"

using boost::asio::ip::tcp;
namespace http = boost::beast::http;
using namespace std::placeholders; 

mjpeg_server::mjpeg_server(unsigned short port, const std::string& path)
: _ioc() 
, _http_server(_ioc, port, std::bind(&mjpeg_server::http_connection, this, _1))
, _stream_path(path)
{
    _thread = std::thread(std::bind(&mjpeg_server::work_thread, this));
}

mjpeg_server::~mjpeg_server()
{
    _ioc.stop(); 
}

void mjpeg_server::work_thread() 
{
    _ioc.run(); 
}

// do nothing in callback but keep connection 
void mjpeg_server::http_connection(tcp::socket socket)
{
    std::cout << "http connection" << std::endl; 
    auto session = std::make_shared<http_session>(std::move(socket), _stream_path); 
    if (session) {
        std::unique_lock<std::mutex> lock(_mutex); 
        _sessions.push_back(session);
    }
}

void mjpeg_server::update_image(unsigned char* buffer, size_t size, int id)
{
    std::unique_lock<std::mutex> lock(_mutex);
    auto it = _sessions.begin(); 
    while (it != _sessions.end()) {
        if ((*it)->live()) {
            if ((*it)->streaming() && (*it)->stream_id() == id) {
                (*it)->update_image(buffer, size); 
            }    
            ++it;
        }
        else {
            it = _sessions.erase(it); 
        }
    } 
}
