#include "replay_trajectory.h"
#include <iostream>

void check(bool value) { if (!value) throw std::runtime_error("Test failed"); }
template <typename F> void rejects(F f) {
    bool rejected=false;
    try { f(); } catch (const std::exception&) { rejected=true; }
    check(rejected);
}
int main() {
    replay::Trajectory t;
    t.points={ {0,{{0,0,0,0,0,0,0}}}, {2,{{0.1,0.1,0.1,0.1,0.1,0.1,0.1}}},
               {4,{{0.2,0.2,0.2,0.2,0.2,0.2,0.2}}} };
    t.calculateSlopes();
    replay::Limits l;
    l.blend=2;
    l.lower.fill(-1); l.upper.fill(1); l.velocity.fill(0.2); l.acceleration.fill(0.5);
    t.validate(l);
    check(t.sample(-1)[0]==0 && t.sample(10)[0]==0.2);
    check(std::abs(t.sample(2)[0]-0.1)<1e-12);
    check(std::abs((t.sample(0.00001)[0]-t.sample(0)[0])/0.00001)<0.00001);
    check(std::abs((t.sample(4)[0]-t.sample(3.99999)[0])/0.00001)<0.00001);
    for (double s=0.001;s<3.999;s+=0.01) {
        const auto q=t.sample(s);
        const double v=(t.sample(s+0.0001)[0]-t.sample(s-0.0001)[0])/0.0002;
        check(q[0]>=0 && q[0]<=0.2 && std::abs(v)<=0.2);
    }
    replay::Joints current{}; current.fill(-0.01);
    const auto blended=t.withStart(current,2);
    blended.validate(l);
    check(blended.sample(0)[0]==-0.01 && blended.sample(2)[0]==0);
    check(blended.duration()==6);
    auto tight=l; tight.upper.fill(0.15);
    rejects([&](){t.validate(tight);});
    tight=l; tight.velocity.fill(0.01);
    rejects([&](){t.validate(tight);});
    tight=l; tight.acceleration.fill(0.001);
    rejects([&](){t.validate(tight);});
    // Waypoints are in bounds, but interpolated control points exceed them.
    auto overshoot=t;
    overshoot.slopes[1].fill(5);
    rejects([&](){overshoot.validate(l);});
    // Automatic approach: stationary endpoints, no overshoot, synchronized joints,
    // and bounded derivatives for both directions and an unchanged joint.
    replay::Joints from{{-0.8,0.8,0,0.1,-0.2,0.2,0}};
    replay::Joints to{{0.8,-0.4,0,0.2,0.2,-0.2,0.5}};
    const auto approach=replay::Trajectory::approach(from,to,l);
    approach.validate(l);
    check(approach.sample(0)==from && approach.sample(approach.duration())==to);
    for (double time=0;time<=approach.duration();time+=0.01) {
        const auto q=approach.sample(time);
        for (size_t j=0;j<7;++j) {
            check(q[j]>=std::min(from[j],to[j])-1e-12 && q[j]<=std::max(from[j],to[j])+1e-12);
            if (time>0.001 && time<approach.duration()-0.001) {
                const double e=0.0001;
                const double before=approach.sample(time-e)[j], after=approach.sample(time+e)[j];
                check(std::abs((after-before)/(2*e))<=l.velocity[j]+1e-6);
                check(std::abs((after-2*q[j]+before)/(e*e))<=l.acceleration[j]+1e-5);
            }
        }
    }
    check(replay::Trajectory::approach(from,from,l).sample(1)==from);
    auto outside=from; outside[0]=-2;
    rejects([&](){replay::Trajectory::approach(outside,to,l);});
    outside=to; outside[1]=2;
    rejects([&](){replay::Trajectory::approach(from,outside,l);});
    auto slower=l; slower.velocity.fill(0.05);
    check(replay::Trajectory::approach(from,to,slower).duration()>approach.duration());
    std::cout<<"Replay interpolation, endpoints, blend, and limit rejection passed.\n";
}
