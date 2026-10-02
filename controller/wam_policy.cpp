#include <chrono>
#include <csignal>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <thread>
#include <array>
#include <algorithm>
#include <deque>
#include <mutex>
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
constexpr uint64_t kDefaultSamplePeriodNs = 10000000ULL;
constexpr uint64_t kStatePeriodNs = 20000000ULL;
constexpr uint64_t kCommandTimeoutNs = 500000000ULL;
volatile std::sig_atomic_t interrupted = 0;
void onSignal(int) { interrupted = 1; }
}

#pragma pack(push, 1)
struct PolicyStatePacket {
    char magic[4];
    uint32_t version;
    uint64_t sequence;
    uint64_t timestamp_ns;
    uint64_t remaining_ns;
    uint64_t next_action_index;
    double positions[7];
    double velocities[7];
};

struct ActionChunkPacket {
    char magic[4];
    uint32_t version;
    uint64_t first_index;
    uint64_t sample_period_ns;
    uint32_t count;
    double positions[7 * kMaxCommandHorizon];
};
#pragma pack(pop)
static_assert(sizeof(PolicyStatePacket) == 152, "Policy state packet layout mismatch");
static_assert(sizeof(ActionChunkPacket) == 476, "Action chunk packet layout mismatch");

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

    const int command_port = 6561;
    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) throw std::runtime_error("Cannot create UDP socket");
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port = htons(static_cast<uint16_t>(command_port));
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    if (::bind(fd, reinterpret_cast<sockaddr*>(&local), sizeof(local)) != 0) {
        ::close(fd);
        throw std::runtime_error("Cannot bind UDP listener port");
    }

    const char* python_host = std::getenv("WAM_POLICY_HOST");
    const char* state_port_env = std::getenv("WAM_POLICY_STATE_PORT");
    const int state_port = state_port_env ? std::stoi(state_port_env) : 6562;
    if (state_port < 1 || state_port > 65535) {
        ::close(fd);
        throw std::runtime_error("Invalid WAM_POLICY_STATE_PORT");
    }
    sockaddr_in state_target{};
    state_target.sin_family = AF_INET;
    state_target.sin_port = htons(static_cast<uint16_t>(state_port));
    if (::inet_pton(AF_INET, python_host ? python_host : "127.0.0.1", &state_target.sin_addr) != 1) {
        ::close(fd);
        throw std::runtime_error("WAM_POLICY_HOST must be an IPv4 address");
    }
    const int state_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (state_fd < 0) {
        ::close(fd);
        throw std::runtime_error("Cannot create UDP state socket");
    }

    using Joints = std::array<double, 7>;
    std::deque<Joints> action_buffer;
    std::mutex buffer_mutex;
    Joints hold{};
    const auto initial_position = wam.getJointPositions();
    for (size_t j = 0; j < 7; ++j) hold[j] = initial_position[j];
    uint64_t sample_period_ns = kDefaultSamplePeriodNs;
    uint64_t next_action_index = 0;
    uint64_t state_sequence = 0;
    uint64_t last_receive_ns = 0;
    uint64_t last_state_send_ns = 0;
    auto next_sample_time = std::chrono::steady_clock::now();
    bool sample_clock_started = false;
    bool tracking = false;

    barrett::systems::Callback<double, jp_type> reference([&](const double&) -> jp_type {
        jp_type out;
        const auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(buffer_mutex);
        if (action_buffer.empty()) {
            for (size_t j = 0; j < 7; ++j) out[j] = hold[j];
            return out;
        }

        const auto sample_period = std::chrono::nanoseconds(sample_period_ns);
        if (!sample_clock_started) {
            next_sample_time = now;
            sample_clock_started = true;
        }
        while (!action_buffer.empty() && now >= next_sample_time + sample_period) {
            hold = action_buffer.front();
            action_buffer.pop_front();
            next_sample_time += sample_period;
        }
        if (action_buffer.empty()) {
            sample_clock_started = false;
            for (size_t j = 0; j < 7; ++j) out[j] = hold[j];
            return out;
        }

        const double elapsed = std::chrono::duration<double>(now - next_sample_time).count();
        const double alpha = std::min(1.0, std::max(0.0, elapsed * 1e9 / static_cast<double>(sample_period_ns)));
        for (size_t j = 0; j < 7; ++j) {
            if (action_buffer.size() > 1) {
                out[j] = action_buffer[0][j] + (action_buffer[1][j] - action_buffer[0][j]) * alpha;
            } else {
                out[j] = action_buffer.front()[j];
            }
        }
        return out;
    });

    barrett::systems::Ramp clock(pm.getExecutionManager(), 1.0);
    clock.stop();
    clock.reset();
    barrett::systems::connect(clock.output, reference.input);
    wam.gravityCompensate();
    std::cout << "Gravity compensation active. Guide the WAM to the demo start pose.\n"
              << "Policy chunks: UDP port " << command_port << "; state feedback: "
              << (python_host ? python_host : "127.0.0.1") << ":" << state_port << ".\n";

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
                    std::array<char, sizeof(ActionChunkPacket)> buffer{};
                    sockaddr_in sender{};
                    socklen_t sender_len = sizeof(sender);
                    const auto n = ::recvfrom(fd, buffer.data(), buffer.size(), 0,
                                              reinterpret_cast<sockaddr*>(&sender), &sender_len);
                    if (n != sizeof(ActionChunkPacket)) continue;
                    const auto* packet = reinterpret_cast<const ActionChunkPacket*>(buffer.data());
                    if (std::memcmp(packet->magic, "WAMH", 4) != 0 || packet->version != 1) continue;
                    if (packet->count == 0 || packet->count > kMaxCommandHorizon
                        || packet->sample_period_ns < 1000000ULL || packet->sample_period_ns > 1000000000ULL) continue;

                    bool valid = true;
                    for (size_t i = 0; i < packet->count; ++i) {
                        for (size_t j = 0; j < 7; ++j) {
                            const double value = packet->positions[i * 7 + j];
                            if (!std::isfinite(value)) valid = false;
                        }
                    }
                    if (!valid) continue;

                    bool accepted = false;
                    {
                        std::lock_guard<std::mutex> lock(buffer_mutex);
                        if (packet->first_index != next_action_index) {
                            if (packet->first_index > next_action_index) {
                                std::cerr << "Action chunk gap: expected index " << next_action_index
                                          << ", got " << packet->first_index << ".\n";
                            }
                        } else if (next_action_index != 0 && packet->sample_period_ns != sample_period_ns) {
                            std::cerr << "Ignoring chunk with changed sample period.\n";
                        } else {
                            sample_period_ns = packet->sample_period_ns;
                            if (action_buffer.empty()) {
                                next_sample_time = std::chrono::steady_clock::now();
                                sample_clock_started = true;
                            }
                            for (size_t i = 0; i < packet->count; ++i) {
                                Joints action{};
                                for (size_t j = 0; j < 7; ++j) action[j] = packet->positions[i * 7 + j];
                                action_buffer.push_back(action);
                            }
                            next_action_index += packet->count;
                            last_receive_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                std::chrono::steady_clock::now().time_since_epoch()).count();
                            accepted = true;
                        }
                    }
                    if (accepted) {
                        if (!tracking) {
                            wam.idle();
                            wam.trackReferenceSignal(reference.output);
                            clock.start();
                            tracking = true;
                            std::cout << "Policy tracking started at action index " << packet->first_index << ".\n";
                        }
                    }
                }
            }

            const auto now = std::chrono::steady_clock::now();
            const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
            if (now_ns - last_state_send_ns >= static_cast<int64_t>(kStatePeriodNs)) {
                const auto q = wam.getJointPositions();
                const auto dq = wam.getJointVelocities();
                PolicyStatePacket state{};
                std::memcpy(state.magic, "WAPS", 4);
                state.version = 1;
                state.sequence = state_sequence;
                state.timestamp_ns = static_cast<uint64_t>(now_ns);
                state.next_action_index = 0;
                {
                    std::lock_guard<std::mutex> lock(buffer_mutex);
                    state.next_action_index = next_action_index;
                    if (!action_buffer.empty()) {
                        const auto age = sample_clock_started && now > next_sample_time
                            ? static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now - next_sample_time).count())
                            : 0ULL;
                        const uint64_t queued_ns = static_cast<uint64_t>(action_buffer.size()) * sample_period_ns;
                        state.remaining_ns = queued_ns > age ? queued_ns - age : 0;
                    }
                }
                for (size_t j = 0; j < 7; ++j) {
                    state.positions[j] = q[j];
                    state.velocities[j] = dq[j];
                }
                if (::sendto(state_fd, &state, sizeof(state), MSG_DONTWAIT,
                             reinterpret_cast<sockaddr*>(&state_target), sizeof(state_target)) == sizeof(state)) {
                    ++state_sequence;
                }
                last_state_send_ns = now_ns;
            }

            if (tracking && last_receive_ns != 0 && now_ns - static_cast<int64_t>(last_receive_ns) > static_cast<int64_t>(kCommandTimeoutNs)) {
                bool empty = false;
                {
                    std::lock_guard<std::mutex> lock(buffer_mutex);
                    empty = action_buffer.empty();
                }
                if (empty) {
                    std::cout << "Action buffer empty and chunk stream timed out; returning to gravity compensation.\n";
                    clock.stop();
                    wam.idle();
                    wam.gravityCompensate();
                    tracking = false;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    } catch (...) {
        stop();
        ::close(fd);
        ::close(state_fd);
        throw;
    }

    stop();
    std::cout << "Policy mode stopped. Shift-idle the arm to finish shutdown.\n";
    pm.getSafetyModule()->waitForMode(barrett::SafetyModule::IDLE);
    ::close(fd);
    ::close(state_fd);
    return 0;
}
