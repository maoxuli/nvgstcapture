#ifndef __MJPEG_SERVER_H
#define __MJPEG_SERVER_H

#include <iostream>
#include <cstdlib>
#include <string>
#include <memory>
#include <deque>

#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/config.hpp>
#include <boost/exception/exception.hpp>

using boost::asio::ip::tcp;
namespace http = boost::beast::http;

#define NUM_BUFFERS 5

// https://localhost:8090/stream?id=0&fps=15
class mjpeg_server
{
public: 
    mjpeg_server(unsigned short port = 8090, const std::string& path = "stream"); 
    ~mjpeg_server(); 

    void update_image(unsigned char* buffer, size_t size, int id = 0);  

private: 
    // http session for streaming 
    // working in separate thread 
    class http_session 
    {
    public:
        http_session(tcp::socket socket, const std::string& path)
        : _socket(std::move(socket))
        , _stream_path(path)
        , _stream_id(0)
        , _streaming(false)
        , _live(true)
        {
            // init buffer pool 
            _pool.resize(NUM_BUFFERS); 

            std::cout << "Starting HTTP session thread" << std::endl; 
            _thread = std::thread(std::bind(&http_session::work_thread, this));
        }

        ~http_session() 
        {         
            std::cout << "Stopping HTTP session thead" << std::endl; 
            _streaming = false;
            _live = false; 
            _images_cond.notify_all();
            _thread.join(); 
            std::cout << "HTTP session thread stopped" << std::endl; 
        }

        // state 
        int stream_id() const { return _stream_id; }
        bool streaming() const { return _streaming; }
        bool live() const { return _live; }

        // push image into queue 
        // Todo: if queue is full (pool is empty), discard a frame 
        // in queue every two frames 
        void update_image(unsigned char* buffer, size_t size)
        {
            if (!_live || !_streaming) return; 

            // find a buffer from pool 
            std::vector<unsigned char> image;
            std::unique_lock<std::mutex> pool_lock(_pool_mutex); 
            if (_pool.empty()) {
                std::cout << "Empty pool, skip frame!!!" << std::endl; 
                return; 
            }
            _pool.front().swap(image); 
            _pool.pop_front(); 
            pool_lock.unlock(); 

            // copy image to buffer 
            image.resize(size); 
            memcpy(image.data(), buffer, size); 

            // move the image buffer to queue 
            std::unique_lock<std::mutex> images_lock(_images_mutex); 
            _images.emplace_back(); 
            _images.back().swap(image); 
            images_lock.unlock();
            _images_cond.notify_one(); 
        }

    private: 
        void work_thread() 
        {
            std::cout << "HTTP session thread in" << std::endl; 
            assert(_live); 

            // establish streaming 
            boost::system::error_code ec;
            boost::beast::flat_buffer buf;
            http::request<http::string_body> req;
            std::cout << "Reading HTTP request" << std::endl; 
            http::read(_socket, buf, req, ec);
            if(ec) {
                std::cerr << ec.message() << std::endl;
                _live = false; 
                return; 
            } 

            std::string target(req.target()); 
            std::cout << "target: " << target << std::endl; 
            
            std::string path; 
            size_t start, stop; 
            start = target.find_first_not_of('/'); 
            if (start != std::string::npos) {
                stop = target.find_first_of('?', start); 
                if (stop == std::string::npos) 
                    path = target.substr(start); 
                else 
                    path = target.substr(start, stop - start); 
            }
            std::cout << "path: " << path << std::endl; 
            
            if (path != _stream_path) {
                std::cout << "Not stream request!" << std::endl; 
                _live = false; 
                return; 
            }

            int id = 0;
            start = target.find("id", stop+1); 
            if (start != std::string::npos) {
                start = target.find_first_of('=', start); 
                if (start != std::string::npos) {
                    stop = target.find_first_of('&', start); 
                    if (stop != std::string::npos) {
                        std::string id_str = target.substr(start+1, stop-start-1);
                        id = atoi(id_str.c_str());
                    } 
                    else {
                        std::string id_str = target.substr(start+1);
                        id = atoi(id_str.c_str());
                    }
                }
            }
            _stream_id = id; 
            std::cout << "id=" << _stream_id << std::endl; 

            try {
                http::response<http::empty_body> res{http::status::ok, req.version()};
                res.set(http::field::server, BOOST_BEAST_VERSION_STRING);
                res.set(http::field::content_type, "multipart/x-mixed-replace; boundary=frame");
                res.keep_alive();
                http::response_serializer<http::empty_body> sr{res};
                std::cout << "Sending HTTP response header" << std::endl; 
                http::write_header(_socket, sr);
                _streaming = true; 
            }
            catch(boost::system::system_error& ex) {
                std::cerr << ex.what() << std::endl;
                _live = false; 
                return;
            }

            // streaming 
            std::vector<unsigned char> image; 
            std::unique_lock<std::mutex> pool_lock(_pool_mutex, std::defer_lock); 
            std::unique_lock<std::mutex> images_lock(_images_mutex); 
            while (_live && _streaming)
            {
                if (_images.empty()) {
                    _images_cond.wait(images_lock); 
                    continue; 
                }
                // move the memory from queue 
                _images.back().swap(image); 
                _images.pop_back();
                images_lock.unlock();

                // sending image 
                auto const len = image.size();
                if (len > 0) {
                    http::response<http::vector_body<unsigned char> > res{std::piecewise_construct,
                        std::make_tuple(std::move(image)), std::make_tuple(http::status::ok, req.version())};
                    res.set(http::field::body, "--frame");
                    res.set(http::field::server, BOOST_BEAST_VERSION_STRING);
                    res.set(http::field::content_type, "image/jpeg");
                    res.content_length(len);
                    res.keep_alive(req.keep_alive());
                    http::write(_socket, res, ec);
                    if(ec) {
                        std::cerr << ec.message() << std::endl;
                        _streaming = false; 
                    } 
                }

                // move the memory to pool 
                pool_lock.lock(); 
                _pool.emplace_back(); 
                _pool.back().swap(image); 
                pool_lock.unlock();
                images_lock.lock();
            }
            images_lock.unlock();
            std::cout << "HTTP session thread out" << std::endl;  
        }

    private: 
        tcp::socket _socket;
        std::string _stream_path; 
        std::atomic<int> _stream_id; 
        std::atomic<bool> _streaming; 
        std::atomic<bool> _live; 
        std::thread _thread; 

        std::deque<std::vector<unsigned char> > _pool; 
        std::mutex _pool_mutex; 

        std::deque<std::vector<unsigned char> > _images; 
        std::mutex _images_mutex; 
        std::condition_variable _images_cond; 
    };

    // tcp server 
    // callback when connection established 
    class tcp_server
    {
    public:
        typedef std::function<void (tcp::socket)> connection_callback;

        tcp_server(boost::asio::io_context& ioc, unsigned short port, 
                   const connection_callback& cb)
        : _acceptor(ioc, tcp::endpoint(tcp::v4(), port))
        , _socket(ioc)
        , _connection_cb(cb)
        {
            std::cout << "Listening on: " << port << std::endl; 
            do_accept();
        }

    private:
        void do_accept()
        {
            std::cout << "Aync accept" << std::endl; 
            _acceptor.async_accept(_socket,
                [this](boost::system::error_code ec)
                {
                    std::cout << "Accepted" << std::endl; 
                    if (!ec && _connection_cb) {
                        _connection_cb(std::move(_socket)); 
                    }
                    do_accept();
                });
        }

        tcp::acceptor _acceptor;
        tcp::socket _socket;
        connection_callback _connection_cb; 
    };

private: 
    boost::asio::io_context _ioc;
    tcp_server _http_server; 
    std::string _stream_path; 

    std::vector<std::shared_ptr<http_session> > _sessions; 
    std::mutex _mutex; 

    std::thread _thread;
    void work_thread(); 
    void http_connection(tcp::socket socket); 
}; 

#endif 
