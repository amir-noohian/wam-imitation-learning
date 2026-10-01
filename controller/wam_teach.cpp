// Standalone, state-only kinesthetic teaching. No ROS, handle, or gripper.
#include <barrett/products/product_manager.h>
#include <barrett/systems.h>
#include <barrett/units.h>
#include <barrett/standard_main_function.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <poll.h>
#include <unistd.h>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <thread>

#pragma pack(push, 1)
struct StatePacket {
    char magic[4];
    uint32_t version;
    uint64_t sequence;
    uint64_t timestamp_ns;
    double positions[7];
    double velocities[7];
};
#pragma pack(pop)
static_assert(sizeof(StatePacket) == 136, "State packet layout mismatch");

struct Socket {
    int fd;
    Socket() : fd(::socket(AF_INET, SOCK_DGRAM, 0)) {
        if (fd < 0) throw std::runtime_error("Cannot create UDP socket");
    }
    ~Socket() { ::close(fd); }
};

template <size_t DOF>
int wam_main(int argc, char** argv, barrett::ProductManager& pm,
             barrett::systems::Wam<DOF>& wam) {
    if (DOF != 7) throw std::runtime_error("Only a seven-joint WAM is supported");
    const uint32_t endian = 1;
    if (*reinterpret_cast<const char*>(&endian) != 1 || sizeof(double) != 8)
        throw std::runtime_error("Wire format requires little-endian 64-bit doubles");
    const char* host = std::getenv("WAM_STATE_HOST");
    const char* port_env = std::getenv("WAM_STATE_PORT");
    const int port = port_env ? std::stoi(port_env) : 6560;
    if (port < 1 || port > 65535) throw std::runtime_error("Invalid UDP port");
    sockaddr_in target{};
    target.sin_family = AF_INET;
    target.sin_port = htons(static_cast<uint16_t>(port));
    if (::inet_pton(AF_INET, host ? host : "127.0.0.1", &target.sin_addr) != 1)
        throw std::runtime_error("WAM_STATE_HOST must be an IPv4 address");
    Socket socket;
    StatePacket packet{};
    std::memcpy(packet.magic, "WAM7", 4);
    packet.version = 1;

    // Uses the WAM's installed gravity calibration, without a position reference.
    wam.gravityCompensate();
    std::cout << "Teaching mode: gravity compensation, no commanded trajectory.\n"
              << "Publishing at nominal 500 Hz. Enter q to stop publishing, "
              << "then shift-idle the arm to exit.\n";
    auto next = std::chrono::steady_clock::now();
    size_t send_errors = 0;
    while (true) {
        pollfd input{STDIN_FILENO, POLLIN, 0};
        if (::poll(&input, 1, 0) > 0) {
            if (input.revents & (POLLHUP | POLLERR)) break;
            if (input.revents & POLLIN) {
                char command[64];
                const auto n = ::read(STDIN_FILENO, command, sizeof(command));
                if (n <= 0 || std::memchr(command, 'q', n)) break;
            }
        }
        const auto q = wam.getJointPositions();
        const auto dq = wam.getJointVelocities();
        packet.timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        for (size_t j = 0; j < 7; ++j) {
            packet.positions[j] = q[j];
            packet.velocities[j] = dq[j];
        }
        if (::sendto(socket.fd, &packet, sizeof(packet), MSG_DONTWAIT,
                     reinterpret_cast<sockaddr*>(&target), sizeof(target)) != sizeof(packet))
            ++send_errors;
        ++packet.sequence;
        next += std::chrono::milliseconds(2);
        if (next < std::chrono::steady_clock::now()) next = std::chrono::steady_clock::now();
        std::this_thread::sleep_until(next);
    }
    std::cout << "Publishing stopped; send errors: " << send_errors
              << ". Shift-idle the arm to exit.\n";
    pm.getSafetyModule()->waitForMode(barrett::SafetyModule::IDLE);
    return 0;
}
