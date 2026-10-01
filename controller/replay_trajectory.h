#pragma once
// Hardware-independent parsing, interpolation, and complete-segment limit checks.
#include <array>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <libconfig.h++>

namespace replay {
using Joints = std::array<double, 7>;
struct Point { double time; Joints q; };
struct Limits {
    Joints lower, upper, velocity, acceleration;
    double start, tracking, still, blend;
};
inline void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
inline Joints readArray(const libconfig::Config& config, const char* key) {
    const auto& a = config.lookup(key);
    require(a.getLength() == 7, std::string(key) + " must contain seven approved values");
    Joints out;
    for (size_t j=0; j<7; ++j) {
        out[j] = static_cast<double>(a[static_cast<int>(j)]);
        require(std::isfinite(out[j]), "Non-finite limit");
    }
    return out;
}
inline Limits readLimits(const char* filename) {
    libconfig::Config c;
    c.setAutoConvert(true);
    c.readFile(filename);
    Limits l;
    l.lower=readArray(c,"q_min"); l.upper=readArray(c,"q_max");
    l.velocity=readArray(c,"max_velocity"); l.acceleration=readArray(c,"max_acceleration");
    l.start=c.lookup("start_tolerance"); l.tracking=c.lookup("tracking_tolerance");
    l.still=c.lookup("still_velocity"); l.blend=c.lookup("start_blend_seconds");
    for (size_t j=0; j<7; ++j)
        require(l.lower[j]<l.upper[j] && l.velocity[j]>0 && l.acceleration[j]>0,
                "Invalid position, velocity, or acceleration limits");
    for (double value : {l.start,l.tracking,l.still,l.blend})
        require(std::isfinite(value) && value>0, "Tolerances and blend duration must be positive");
    return l;
}

class Trajectory {
public:
    std::vector<Point> points;
    std::vector<Joints> slopes;
    static Trajectory load(const char* filename) {
        std::ifstream file(filename);
        require(bool(file), "Cannot open trajectory CSV");
        std::string line;
        std::getline(file,line);
        require(line=="time_s,j1,j2,j3,j4,j5,j6,j7", "Unexpected trajectory CSV header");
        Trajectory t;
        while (std::getline(file,line)) {
            require(t.points.size()<1000000, "Trajectory exceeds one million samples");
            std::replace(line.begin(),line.end(),',',' ');
            std::istringstream row(line);
            Point p;
            require(bool(row>>p.time) && std::isfinite(p.time), "Invalid sample time");
            for (double& q : p.q) require(bool(row>>q) && std::isfinite(q), "Invalid joint position");
            std::string extra;
            require(!(row>>extra), "Unexpected extra CSV column");
            if (!t.points.empty()) require(p.time>t.points.back().time, "Sample times must increase");
            t.points.push_back(p);
        }
        require(t.points.size()>=3 && t.points.front().time==0, "Need >=3 samples starting at zero");
        require(t.points.back().time<=3600, "Trajectory exceeds one hour");
        t.calculateSlopes();
        return t;
    }
    void calculateSlopes() {
        slopes.assign(points.size(), Joints{}); // zero velocity at both endpoints
        for (size_t i=1; i+1<points.size(); ++i)
            for (size_t j=0; j<7; ++j)
                slopes[i][j]=(points[i+1].q[j]-points[i-1].q[j])/(points[i+1].time-points[i-1].time);
    }
    static Trajectory approach(const Joints& measured, const Joints& goal, const Limits& l) {
        // A single zero-end-velocity cubic segment. Select duration using the
        // same conservative Bezier derivative bounds used by validate().
        double seconds=l.blend;
        for (size_t j=0;j<7;++j) {
            require(std::isfinite(measured[j]) && std::isfinite(goal[j]), "Non-finite approach posture");
            const double distance=std::abs(goal[j]-measured[j]);
            seconds=std::max(seconds,1.1*3*distance/l.velocity[j]);
            seconds=std::max(seconds,1.1*std::sqrt(6*distance/l.acceleration[j]));
        }
        require(std::isfinite(seconds) && seconds>0 && seconds<=3600, "Invalid approach duration");
        Trajectory result;
        result.points={Point{0,measured},Point{seconds,goal}};
        result.slopes.assign(2,Joints{});
        result.validate(l);
        return result;
    }
    Trajectory withStart(const Joints& measured, double seconds) const {
        Trajectory result=*this;
        for (auto& p : result.points) p.time+=seconds;
        result.points.insert(result.points.begin(),Point{0,measured});
        // Preserve a zero derivative on each side of the start blend.
        result.slopes.insert(result.slopes.begin(),Joints{});
        return result;
    }
    void validate(const Limits& l) const {
        for (size_t i=0; i+1<points.size(); ++i) {
            const double h=points[i+1].time-points[i].time;
            for (size_t j=0; j<7; ++j) {
                // Cubic Bezier control points give conservative bounds over the
                // entire Hermite segment, including between sampled waypoints.
                const double b[]={points[i].q[j],points[i].q[j]+h*slopes[i][j]/3,
                                  points[i+1].q[j]-h*slopes[i+1][j]/3,points[i+1].q[j]};
                double v[3];
                for (double q : b)
                    require(q>=l.lower[j] && q<=l.upper[j], "Position bound exceeded at joint "+std::to_string(j+1));
                for (size_t k=0;k<3;++k) {
                    v[k]=3*(b[k+1]-b[k])/h;
                    require(std::abs(v[k])<=l.velocity[j], "Velocity bound exceeded; increase slowdown");
                }
                for (size_t k=0;k<2;++k)
                    require(std::abs(2*(v[k+1]-v[k])/h)<=l.acceleration[j],
                            "Acceleration bound exceeded; increase slowdown");
            }
        }
    }
    Joints sample(double time) const {
        if (time<=0) return points.front().q;
        if (time>=duration()) return points.back().q;
        auto it=std::upper_bound(points.begin(),points.end(),time,
                                [](double t,const Point& p){return t<p.time;});
        const size_t i=static_cast<size_t>(it-points.begin()-1);
        const double h=points[i+1].time-points[i].time;
        const double u=(time-points[i].time)/h, u2=u*u, u3=u2*u;
        Joints out;
        for (size_t j=0;j<7;++j)
            out[j]=(2*u3-3*u2+1)*points[i].q[j]+(u3-2*u2+u)*h*slopes[i][j]
                  +(-2*u3+3*u2)*points[i+1].q[j]+(u3-u2)*h*slopes[i+1][j];
        return out;
    }
    double duration() const { return points.back().time; }
};
} // namespace replay
