#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <array>
#include <algorithm>
#include <poll.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <barrett/products/product_manager.h>
#include <barrett/systems.h>
#include <barrett/units.h>
#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>

namespace {
constexpr size_t kMaxCommandHorizon = 8;
constexpr double kCommandTimeoutSeconds = 0.25;
constexpr double kNominalStepSeconds = 1.0 / 20.0;
volatile std::sig_atomic_t interrupted = 0;
void onSignal(int) { interrupted = 1; }
}

#pragma pack(push, 1)
struct LegacyCommandPacket {
    char magic[4];
    uint32_t version;
    uint64_t sequence;
    uint64_t timestamp_ns;
    uint32_t horizon_size;
    double positions[7];
};

struct HorizonCommandPacket {
    char magic[4];
    uint32_t version;
    uint64_t sequence;
    uint64_t timestamp_ns;
    uint32_t horizon_size;
    double positions[7 * kMaxCommandHorizon];
};
#pragma pack(pop)
static_assert(sizeof(LegacyCommandPacket) == 84, "Legacy command packet layout mismatch");
static_assert(sizeof(HorizonCommandPacket) == 476, "Horizon command packet layout mismatch");

bool validate_args(int argc, char** argv) {
    (void)argc;
    (void)argv;
    return true;
}

template <size_t DOF>
int wam_main(int argc, char** argv, barrett::ProductManager& pm,
             barrett::systems::Wam<DOF>& wam) {
    if (DOF != 7) throw std::runtime_error("Policy execution requires a seven-joint WAM");
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    const int port = 6561;
    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) throw std::runtime_error("Cannot create UDP socket");
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port = htons(static_cast<uint16_t>(port));
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    if (::bind(fd, reinterpret_cast<sockaddr*>(&local), sizeof(local)) != 0) {
        ::close(fd);
        throw std::runtime_error("Cannot bind UDP listener port");
    }

    std::array<std::array<double, 7>, kMaxCommandHorizon> horizon{};
    std::atomic<uint32_t> horizon_size{1};
    std::atomic<uint64_t> packet_timestamp_ns{0};
    std::atomic<uint64_t> last_receive_ns{0};
    std::atomic<bool> stale{false};
    std::array<double, 7> hold{};
    const auto initial_position = wam.getJointPositions();
    for (size_t j = 0; j < 7; ++j) hold[j] = initial_position[j];

    barrett::systems::Callback<double, jp_type> reference([&](const double&) -> jp_type {
        jp_type out;
        const auto now_ns = std::chrono::steady_clock::now().time_since_epoch().count();
        const auto last_ns = last_receive_ns.load();
        const auto current_horizon_size = std::max<uint32_t>(1u, horizon_size.load());
        if (last_ns == 0 || (now_ns - last_ns) > static_cast<uint64_t>(kCommandTimeoutSeconds * 1e9)) {
            for (size_t j = 0; j < 7; ++j) out[j] = hold[j];
            return out;
        }

        const double elapsed = static_cast<double>(now_ns - packet_timestamp_ns.load()) * 1e-9;
        const double step = std::max(kNominalStepSeconds, 1.0 / 20.0);
        const size_t points = std::min<size_t>(static_cast<size_t>(current_horizon_size), kMaxCommandHorizon);
        if (points == 1) {
            for (size_t j = 0; j < 7; ++j) out[j] = horizon[0][j];
            return out;
        }

        double local = std::max(0.0, elapsed);
        const size_t segment = std::min<size_t>(static_cast<size_t>(local / step), points - 1);
        const double segment_elapsed = std::max(0.0, local - static_cast<double>(segment) * step);
        const double alpha = (points > 1 && step > 0.0) ? std::min(1.0, std::max(0.0, segment_elapsed / step)) : 0.0;
        const size_t next_segment = std::min(points - 1, segment + 1);
        for (size_t j = 0; j < 7; ++j) {
            const double q0 = horizon[segment][j];
            const double q1 = horizon[next_segment][j];
            out[j] = q0 + (q1 - q0) * alpha;
        }
        static int debug_counter = 0;
        if (++debug_counter % 100 == 0) {
            std::cout << "DEBUG ref seg=" << segment << " alpha=" << alpha << " q=";
            for (size_t j = 0; j < 7; ++j) std::cout << out[j] << " ";
            std::cout << "\n";
        }
        return out;
    });

    barrett::systems::Ramp clock(pm.getExecutionManager(), 1.0);
    clock.stop();
    clock.reset();
    barrett::systems::connect(clock.output, reference.input);
    wam.gravityCompensate();
    bool tracking = false;
    std::cout << "Gravity compensation active. Guide the WAM to the demo start pose, then start the UDP sender.\n"
              << "Listening for policy commands on port 6561; q: exit.\n";

    const auto stop = [&]() {
        clock.stop();
        wam.idle();
        barrett::systems::disconnect(reference.input);
    };

    try {
        while (!interrupted) {
            pollfd input{fd, POLLIN, 0};
            if (::poll(&input, 1, 0) > 0) {
                if (input.revents & (POLLHUP | POLLERR)) break;
                if (input.revents & POLLIN) {
                    std::array<char, sizeof(HorizonCommandPacket)> buffer{};
                    sockaddr_in sender{};
                    socklen_t sender_len = sizeof(sender);
                    const auto n = ::recvfrom(fd, buffer.data(), buffer.size(), 0,
                                              reinterpret_cast<sockaddr*>(&sender), &sender_len);
                    if (n <= 0) continue;

                    bool accepted = false;
                    if (n == sizeof(LegacyCommandPacket)) {
                        const auto* packet = reinterpret_cast<const LegacyCommandPacket*>(buffer.data());
                        if (std::memcmp(packet->magic, "WAMC", 4) != 0) continue;
                        if (packet->version != 1) continue;
                        const auto size = std::max<uint32_t>(1u, packet->horizon_size);
                        horizon_size.store(size);
                        packet_timestamp_ns.store(packet->timestamp_ns);
                        last_receive_ns.store(std::chrono::steady_clock::now().time_since_epoch().count());
                        std::cout << "DEBUG packet legacy seq=" << packet->sequence << " ts=" << packet->timestamp_ns << " h=" << size
                                  << " q0=" << packet->positions[0] << " " << packet->positions[1] << " " << packet->positions[2] << "\n";
                        for (size_t j = 0; j < 7; ++j) {
                            horizon[0][j] = packet->positions[j];
                            hold[j] = packet->positions[j];
                        }
                        for (size_t i = 1; i < kMaxCommandHorizon; ++i) {
                            for (size_t j = 0; j < 7; ++j) horizon[i][j] = packet->positions[j];
                        }
                        accepted = true;
                    } else if (n == sizeof(HorizonCommandPacket)) {
                        const auto* packet = reinterpret_cast<const HorizonCommandPacket*>(buffer.data());
                        if (std::memcmp(packet->magic, "WAMC", 4) != 0) continue;
                        if (packet->version != 1) continue;
                        const auto size = std::max<uint32_t>(1u, packet->horizon_size);
                        const auto actual_points = std::min<uint32_t>(size, static_cast<uint32_t>(kMaxCommandHorizon));
                        horizon_size.store(actual_points);
                        packet_timestamp_ns.store(packet->timestamp_ns);
                        last_receive_ns.store(std::chrono::steady_clock::now().time_since_epoch().count());
                        std::cout << "DEBUG packet seq=" << packet->sequence << " ts=" << packet->timestamp_ns << " h=" << actual_points
                                  << " q0=" << packet->positions[0] << " " << packet->positions[1] << " " << packet->positions[2] << "\n";
                        for (size_t i = 0; i < kMaxCommandHorizon; ++i) {
                            for (size_t j = 0; j < 7; ++j) {
                                const size_t idx = i * 7 + j;
                                const double value = i < actual_points ? packet->positions[idx] : packet->positions[(actual_points - 1) * 7 + j];
                                horizon[i][j] = value;
                                if (i == 0) hold[j] = value;
                            }
                        }
                        accepted = true;
                    }
                    if (accepted) {
                        stale.store(false);
                        if (!tracking) {
                            wam.idle();
                            wam.trackReferenceSignal(reference.output);
                            clock.start();
                            tracking = true;
                            std::cout << "Policy tracking started from the first received command.\n";
                        }
                    }
                }
            }

            const auto now_ns = std::chrono::steady_clock::now().time_since_epoch().count();
            const auto last_ns = last_receive_ns.load();
            if (last_ns != 0 && (now_ns - last_ns) > static_cast<uint64_t>(kCommandTimeoutSeconds * 1e9) && !stale.load()) {
                std::cout << "UDP command timeout: stopping motion and waiting for a fresh command horizon.\n";
                stale.store(true);
                clock.stop();
                wam.idle();
                wam.gravityCompensate();
                tracking = false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    } catch (...) {
        stop();
        throw;
    }

    stop();
    std::cout << "Policy mode stopped. Shift-idle the arm to finish shutdown.\n";
    pm.getSafetyModule()->waitForMode(barrett::SafetyModule::IDLE);
    ::close(fd);
    return 0;
}
