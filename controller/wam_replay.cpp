// Preload and validate before libbarrett initializes any hardware.
#include "replay_trajectory.h"
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>
#include <poll.h>
#include <unistd.h>
#include <barrett/products/product_manager.h>
#include <barrett/systems.h>
#include <barrett/units.h>
#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>

namespace {
replay::Trajectory trajectory;
replay::Limits limits;
volatile std::sig_atomic_t interrupted=0;
void onSignal(int) { interrupted=1; }
}

bool validate_args(int argc,char** argv) {
    try {
        const bool check=argc==4 && std::string(argv[1])=="--check";
        if (argc!=3 && !check) {
            std::cerr<<"Usage: wam_replay [--check] trajectory.csv replay.conf\n";
            return false;
        }
        trajectory=replay::Trajectory::load(argv[check?2:1]);
        limits=replay::readLimits(argv[check?3:2]);
        trajectory.validate(limits);
        std::cout<<"Validated "<<trajectory.points.size()<<" waypoints, "
                 <<trajectory.duration()<<" seconds (+ start blend).\n";
        if (check) std::exit(0); // Must exit BEFORE ProductManager constructs hardware.
        return true;
    } catch (const std::exception& e) {
        std::cerr<<"Replay rejected: "<<e.what()<<"\n";
    } catch (...) {
        std::cerr<<"Replay rejected: cannot parse trajectory or replay limits.\n";
    }
    return false;
}

template <size_t DOF>
int wam_main(int argc,char** argv,barrett::ProductManager& pm,barrett::systems::Wam<DOF>& wam) {
    if (DOF!=7) throw std::runtime_error("Replay requires a seven-joint WAM");
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    wam.gravityCompensate();
    std::signal(SIGINT,onSignal);
    std::signal(SIGTERM,onSignal);
    std::atomic<double> progress{0}, heartbeat{0};
    std::atomic<bool> stalled{false};
    replay::Trajectory active=trajectory;
    barrett::systems::Ramp clock(pm.getExecutionManager(),1.0);
    clock.stop();
    clock.reset();
    barrett::systems::Callback<double,jp_type> reference([&](const double& t)->jp_type {
        // Freeze progression if the supervisory loop stops responding for 250ms.
        double effective=t;
        if (stalled.load() || t-heartbeat.load()>0.25) {
            stalled.store(true);
            effective=progress.load();
        }
        progress.store(effective);
        const auto q=active.sample(effective);
        jp_type out;
        for (size_t j=0;j<7;++j) out[j]=q[j];
        return out;
    });
    bool tracking=false, finished=false, approaching=false, returningHome=false;
    // Disconnect before destroying reference/clock even if an exception occurs.
    struct IdleOnExit {
        barrett::systems::Wam<DOF>& robot;
        ~IdleOnExit() { try { robot.idle(); } catch (...) {} }
    } cleanup{wam};
    std::cout<<"Gravity compensation. h + Enter: move slowly to start; r + Enter: replay.\n"
             <<"b + Enter: move to home.\n"
             <<"s + Enter: stop and return to gravity compensation. q: exit.\nStart (rad): ";
    for (double q : trajectory.points.front().q) std::cout<<q<<" ";
    std::cout<<std::endl;
    const auto stop=[&]() {
        clock.stop();
        wam.idle();
        barrett::systems::disconnect(reference.input);
        tracking=false;
        finished=false;
    };
    try {
        while (!interrupted) {
            pollfd input{STDIN_FILENO,POLLIN,0};
            if (::poll(&input,1,0)>0) {
                if (input.revents & (POLLHUP|POLLERR)) break;
                if (input.revents & POLLIN) {
                    char buffer[128];
                    const auto n=::read(STDIN_FILENO,buffer,sizeof(buffer));
                    if (n<=0) break;
                    const std::string command(buffer,static_cast<size_t>(n));
                    if (command.find('q')!=std::string::npos) break;
                    if (command.find('s')!=std::string::npos) {
                        stop();
                        std::cout<<"Stopped: gravity compensation.\n";
                    } else if (command.find('h')!=std::string::npos || command.find('b')!=std::string::npos
                               || command.find('r')!=std::string::npos) {
                        const bool goToStart=command.find('h')!=std::string::npos;
                        const bool goHome=!goToStart && command.find('b')!=std::string::npos;
                        const bool pointMove=goToStart || goHome;
                        if (tracking && !finished) {
                            std::cout<<"Motion in progress; s stops it before another command.\n";
                            continue;
                        }
                        const auto q=wam.getJointPositions();
                        const auto dq=wam.getJointVelocities();
                        bool stationary=true, near=true;
                        replay::Joints measured;
                        for (size_t j=0;j<7;++j) {
                            measured[j]=q[j];
                            stationary=stationary && std::isfinite(q[j]) && std::isfinite(dq[j])
                                && std::abs(dq[j])<=limits.still;
                            near=near && std::abs(q[j]-trajectory.points.front().q[j])<=limits.start;
                        }
                        if (!stationary || (!pointMove && !near)) {
                            std::cout<<"Start rejected: arm must be stationary; replay also requires proximity to start.\n";
                            continue;
                        }
                        replay::Trajectory candidate;
                        jp_type target;
                        if (goHome) target=wam.getHomePosition();
                        else
                            for (size_t j=0;j<7;++j) target[j]=trajectory.points.front().q[j];
                        try {
                            if (pointMove) {
                                // moveTo follows the joint-space line between these
                                // endpoints, so endpoint bounds cover the path.
                                for (size_t j=0;j<7;++j) {
                                    replay::require(measured[j]>=limits.lower[j] && measured[j]<=limits.upper[j],
                                                    "Measured posture exceeds position limits");
                                    replay::require(std::isfinite(target[j]) && target[j]>=limits.lower[j]
                                                    && target[j]<=limits.upper[j],
                                                    "Target posture exceeds position limits");
                                }
                            } else {
                                candidate=trajectory.withStart(measured,limits.blend);
                                candidate.validate(limits);
                            }
                        } catch (const std::exception& e) {
                            std::cout<<"Start rejected: "<<e.what()<<"\n";
                            continue;
                        }
                        // Stop and disconnect the old reference before changing
                        // data read by the execution-manager callback (including a hold).
                        stop();
                        approaching=pointMove;
                        returningHome=goHome;
                        if (approaching) {
                            // Let libbarrett use its default moveTo velocity/acceleration
                            // rather than the conservative software limits from replay.conf.
                            wam.moveTo(target,false);
                            tracking=true;
                            std::cout<<(returningHome ? "Moving to home" : "Moving to start")
                                     <<" with wam.moveTo().\n";
                            continue;
                        }
                        active=std::move(candidate);
                        progress.store(0); heartbeat.store(0); stalled.store(false);
                        clock.reset();
                        barrett::systems::connect(clock.output,reference.input);
                        wam.trackReferenceSignal(reference.output);
                        tracking=true;
                        clock.start();
                        std::cout<<"Replay started: "<<active.duration()<<" seconds.\n";
                    }
                }
            }
            if (tracking) {
                if (approaching) {
                    if (!finished && wam.moveIsDone()) {
                        finished=true;
                        if (returningHome)
                            std::cout<<"Home reached; holding home posture. h: move to start, s: gravity compensation, q: exit.\n";
                        else
                            std::cout<<"Approach complete; holding start posture. Let arm settle, then r + Enter to replay.\n";
                    }
                } else {
                    const double t=progress.load();
                    heartbeat.store(t);
                    const auto q=wam.getJointPositions();
                    const auto dq=wam.getJointVelocities();
                    replay::Joints desired=active.sample(t);
                    bool fault=stalled.load();
                    for (size_t j=0;j<7;++j)
                        fault=fault || !std::isfinite(q[j]) || !std::isfinite(dq[j])
                            || !std::isfinite(desired[j])
                            || q[j]<limits.lower[j] || q[j]>limits.upper[j]
                            || std::abs(dq[j])>limits.velocity[j]
                            || std::abs(q[j]-desired[j])>limits.tracking;
                    if (fault) {
                        stop();
                        std::cout<<"Motion stopped: state limit, tracking error, or supervisor timeout.\n";
                    } else if (!finished && t>=active.duration()) {
                        clock.stop();
                        finished=true;
                        std::cout<<"Replay complete; holding final position. h: return to start, b: home, s: gravity compensation, q: exit.\n";
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        stop();
    } catch (...) {
        stop();
        throw;
    }
    std::cout<<"Gravity compensation. Shift-idle the arm to finish shutdown.\n";
    pm.getSafetyModule()->waitForMode(barrett::SafetyModule::IDLE);
    return 0;
}
