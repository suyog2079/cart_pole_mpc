#include <arpa/inet.h>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <pthread.h>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

struct recv_data {
  double x;
  double x_dot;
  double theta;
  double theta_dot;
};

struct send_data {
  double u;
};

std::atomic<recv_data> shared_recv;
std::atomic<send_data> shared_send;
std::atomic<int> shared_updated{0};

// ---------------- TCP CLIENT THREAD ----------------
static void *tcp_client(void *arg) {
  printf("TCP thread started\n");

  int a = 0;

reconnect:
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in serverAddr{};
  serverAddr.sin_family = AF_INET;
  serverAddr.sin_port = htons(8080);
  inet_pton(AF_INET, "127.0.0.1", &serverAddr.sin_addr);

  while (connect(sock, (sockaddr *)&serverAddr, sizeof(serverAddr)) < 0) {
    std::cerr << "TCP connect failed, retrying...\n";
    usleep(500000);
  }
  printf("TCP connected\n");

  char buffer[1024];
  std::string stream;

  while (true) {
    // ---- Send command to simulator ----
      send_data s = shared_send.load(std::memory_order_relaxed);
      std::string cmd = std::to_string(s.u) + "\n";
      int sent = send(sock, cmd.c_str(), cmd.size(), 0);
      if (sent < 0) {
        std::cerr << "Send failed\n";
        goto reconnect;
      } else {
        // printf("sent %d: \t %f\n", ++a, s.u);
      }

    usleep(1000);

    // ---- Receive state from simulator ----
    int bytes = recv(sock, buffer, sizeof(buffer) - 1, 0);
    if (bytes <= 0) {
      std::cerr << "Server closed\n";
      goto reconnect;
    }
    buffer[bytes] = '\0';
    stream += buffer;

    // Drain all complete lines, keep only the freshest
    bool got_one = false;
    size_t pos;
    while ((pos = stream.find('\n')) != std::string::npos) {
      std::string line = stream.substr(0, pos);
      stream.erase(0, pos + 1);

      recv_data tmp{};
      // simulator sends: x,theta,xdot,thetadot
      if (sscanf(line.c_str(), "%lf,%lf,%lf,%lf", &tmp.x, &tmp.theta,
                 &tmp.x_dot, &tmp.theta_dot) == 4) {
        // printf("received x: %f \t theta: %f\n", tmp.x, tmp.theta);
        shared_recv.store(tmp, std::memory_order_relaxed);
        shared_updated.store(1, std::memory_order_release);
      }
    }
  }

  close(sock);
  return nullptr;
}
