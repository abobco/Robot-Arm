#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>         
#include <mutex>          
#include <assert.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment( lib, "ws2_32.lib")
#include "common.hpp"

namespace xn{
    static const int MAX_READ = 130272;
    static int arm_state = -2;

    void read_buf(SOCKET sock, char* buf, int size) {
        int rc;
        do {
            rc = recv(sock, buf, size, 0);
            buf += rc;
            size -= rc;
        } while (size > 0);
    }

    void send_int(SOCKET sock, int32_t val) {
        int32_t nval = htonl(val);
        int bs = send(sock, (const char*)&nval, sizeof nval, 0);
        assert(bs == sizeof nval);
    }

    class VideoStream
    {
        std::mutex frame_lock;
        cv::Mat frame_most_recent;
        bool verbose, connected;

    public:
        std::thread read_image_thread;
        cv::VideoCapture cap;
        SOCKET client_sock, server_sock;

        VideoStream() { }
        VideoStream(int port, bool verbose = false) {
            this->verbose = verbose;
            connected = false;

            WSADATA wsa;
            struct sockaddr_in address;
            int opt = 1;
            int addrlen = sizeof address;

            if (verbose) {
                printf("starting TCP socket video stream\n");
                printf("\ninitialising winsock... ");
            }
            if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
                printf("WSAStartup() failed: %d", WSAGetLastError());
                throw - 1;
            }
            if (verbose)
                printf("success\n");

            server_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
            if (server_sock == INVALID_SOCKET) {
                printf("socket() failed: %d", WSAGetLastError());
                throw - 1;
            }

            if (setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt))) {
                perror("setsockopt");
                throw - 1;
            }
            address.sin_family = AF_INET;
            address.sin_addr.s_addr = INADDR_ANY;
            address.sin_port = htons(port);

            if (::bind(server_sock, (struct sockaddr*)&address, sizeof address) < 0) {
                perror("bind failed");
                throw - 1;
            }
            if (listen(server_sock, 3) < 0) {
                perror("listen");
                throw - 1;
            }

            if (verbose)
                printf("waiting for client connection...\n");
            if ((client_sock = accept(server_sock, (struct sockaddr*)&address, (socklen_t*)&addrlen)) < 0) {
                perror("accept");
                throw - 1;
            }

            if (verbose)
                printf("connection accepted\n");
            connected = true;

            read_image_thread = std::thread(&VideoStream::readImageThread, this);
        }

        std::thread& get_read_image_thread() { return read_image_thread;  }

        VideoStream& operator=(const VideoStream& r) {
            //*this = VideoStream(8080, true);
            return *this;
       }

        // REMOVE THIS L8R
        bool isOpened() {
            return true;
        }

        ~VideoStream() {
            connected = false;
            if ( read_image_thread.joinable() )
                read_image_thread.join();
        }

        cv::Mat getLastFrame() {
            cv::Mat frame;
            frame_lock.lock();
            frame = frame_most_recent.clone();
            frame_lock.unlock();
            return frame;
        }

        // TODO: heap allocate this buffer
        void readImageThread() {

            while (connected) {
                //memset(buffer, 0, MAX_READ * sizeof(*buffer));
                char buffer[MAX_READ] = { 0 };
                char* buf;

                // read file size in bytes
                int32_t tmp, left;
                buf = (char*)&tmp;
                read_buf(client_sock, buf, sizeof(tmp));
                left = ntohl(tmp);
                if (verbose)
                    printf("%d byte jpeg\n", left);

                if (left < 0) {
                    arm_state = left;
                    continue;
                }

                // read jpg file into memory
                buf = buffer;
                read_buf(client_sock, buf, left);

                // convert jpg byte array into opencv mat
                cv::Mat rawData(1, sizeof(buffer), CV_8SC1, (void*)buffer);

                frame_lock.lock();
                frame_most_recent = cv::imdecode(rawData, cv::IMREAD_UNCHANGED);
                frame_lock.unlock();
            }
        }

    };

}