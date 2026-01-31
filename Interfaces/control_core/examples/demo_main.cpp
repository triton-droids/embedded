// demo_main.cpp (NEW CONTROL STRUCTURE)
//
// External C++ client that talks to target_gateway_node over HTTP.
// No ROS, no control_core.
//
// Build:
//   g++ -std=c++17 -O2 demo_main.cpp -o demo_main
//
// Usage:
//   ./demo_main 127.0.0.1 8080 shoulder_pitch enable
//   ./demo_main 127.0.0.1 8080 shoulder_pitch velocity 0.8
//
// Sends JSON to: http://HOST:PORT/target

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <sstream>
#include <string>

static int connect_tcp(const std::string& host, int port) {
  struct addrinfo hints;
  std::memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;

  struct addrinfo* res = nullptr;
  const std::string port_s = std::to_string(port);
  if (getaddrinfo(host.c_str(), port_s.c_str(), &hints, &res) != 0) return -1;

  int fd = -1;
  for (auto p = res; p != nullptr; p = p->ai_next) {
    fd = ::socket(p->ai_family, p->ai_socktype, p->ai_protocol);
    if (fd < 0) continue;
    if (::connect(fd, p->ai_addr, p->ai_addrlen) == 0) {
      freeaddrinfo(res);
      return fd;
    }
    ::close(fd);
    fd = -1;
  }

  freeaddrinfo(res);
  return -1;
}

static bool http_post_json(const std::string& host, int port, const std::string& path, const std::string& json_body) {
  int fd = connect_tcp(host, port);
  if (fd < 0) {
    std::cerr << "connect failed\n";
    return false;
  }

  std::ostringstream req;
  req << "POST " << path << " HTTP/1.1\r\n";
  req << "Host: " << host << ":" << port << "\r\n";
  req << "Content-Type: application/json\r\n";
  req << "Accept: application/json\r\n";
  req << "Connection: close\r\n";
  req << "Content-Length: " << json_body.size() << "\r\n";
  req << "\r\n";
  req << json_body;

  const std::string s = req.str();
  if (::send(fd, s.data(), s.size(), 0) < 0) {
    std::cerr << "send failed\n";
    ::close(fd);
    return false;
  }

  std::string resp;
  char buf[4096];
  while (true) {
    ssize_t r = ::recv(fd, buf, sizeof(buf), 0);
    if (r <= 0) break;
    resp.append(buf, buf + r);
  }
  ::close(fd);

  std::cout << resp << "\n";
  return true;
}

static std::string json_escape(const std::string& s) {
  std::string out;
  out.reserve(s.size() + 8);
  for (char c : s) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default: out += c; break;
    }
  }
  return out;
}

int main(int argc, char** argv) {
  if (argc < 5) {
    std::cerr << "Usage:\n"
              << "  " << argv[0] << " HOST PORT JOINT MODE [VALUE]\n"
              << "Modes:\n"
              << "  enable | disable\n"
              << "  velocity VALUE\n";
    return 2;
  }

  const std::string host = argv[1];
  const int port = std::stoi(argv[2]);
  const std::string joint = argv[3];
  const std::string mode = argv[4];

  std::ostringstream body;
  body << "{\"commands\":{\"" << json_escape(joint) << "\":{";
  body << "\"mode\":\"" << json_escape(mode) << "\"";

  if (mode == "velocity") {
    if (argc < 6) {
      std::cerr << "velocity mode needs VALUE\n";
      return 2;
    }
    const double v = std::stod(argv[5]);
    body << ",\"velocity\":" << v;
  }

  body << "}}}";
  return http_post_json(host, port, "/target", body.str()) ? 0 : 1;
}
