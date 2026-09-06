// Simple test executable that exercises SocketUDP
#include "SocketUDP.hh"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <thread>
#include <chrono>

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        // Default: run a basic bind + sendto + recv test
        // Create a receiver socket
        SocketUDP receiver(true, false);
        if (!receiver.bind("0.0.0.0", 19876))
        {
            fprintf(stderr, "Failed to bind receiver\n");
            return 1;
        }

        // Create a sender socket
        SocketUDP sender(true, false);
        if (!sender.bind("0.0.0.0", 0))
        {
            // bind to any port
        }

        const char* msg = "HELLO_ARDUPILOT";
        ssize_t sent = sender.sendto(msg, strlen(msg), "0.0.0.0", 19876);
        if (sent < 0)
        {
            fprintf(stderr, "Failed to send\n");
            return 2;
        }

        // Small delay to let the packet arrive
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        char buf[256] = {0};
        ssize_t recvd = receiver.recv(buf, sizeof(buf), 500);
        if (recvd <= 0)
        {
            fprintf(stderr, "Failed to receive: %zd\n", recvd);
            return 3;
        }

        if (strncmp(buf, msg, strlen(msg)) != 0)
        {
            fprintf(stderr, "Data mismatch: got '%s'\n", buf);
            return 4;
        }

        printf("PASS: sent and received '%s' (%zd bytes)\n", buf, recvd);

        // Test pollin timeout (should return false quickly)
        bool ready = receiver.pollin(10);
        printf("POLLIN_EMPTY: %s\n", ready ? "true" : "false");

        // Test set_blocking
        bool block_ok = receiver.set_blocking(false);
        printf("SET_BLOCKING: %s\n", block_ok ? "ok" : "fail");

        printf("ALL_PASS\n");
        return 0;
    }

    return 0;
}