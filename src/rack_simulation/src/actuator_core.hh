#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <array>

namespace rack_simulation {
inline double Wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
struct Pose { double x{}, y{}, z{}, yaw{}; };
struct Geometry {
  double platformTop{.18}, payloadBottom{.2175}, contact{.0375};
  double up{.30}, down{0.}, upper{.40}, tolerance{.008}, settle{.4};
};

// One state owner, sampled in the simulation thread. Neither a transport ACK
// nor an elapsed timeout is a physical completion. Advance only on observation.
class Payload {
 public:
  Geometry geometry;
  std::string selected, fault;
  bool engaged{false}, carrying{false}, settled{false}, releasing{false};
  double target{0.}, stableSince{-1.}, dx{}, dy{}, dyaw{};
  Pose center;
  void Select(const std::string &name) {
    if (name == selected) return;
    if (engaged || carrying || releasing) { fault = "payload switch while engaged"; return; }
    if (!name.empty() && name != "rack_a" && name != "rack_b" && name != "rack_c" && name != "rack_d") {
      fault = "unknown payload"; return;
    }
    selected = name;
  }
  void Target(double value) {
    if (!std::isfinite(value) || value < geometry.down || value > geometry.upper) {
      fault = "lift command outside range"; return;
    }
    if (std::abs(value-target) > .0001) { stableSince=-1.; settled=false; carrying=false; }
    target=value;
  }
  // Returns a kinematic rack target. The caller applies it in PreUpdate; a
  // later PostUpdate samples the actual rack pose to certify carrying/release.
  bool Follow(const Pose &base, double lift, const Pose &rack, Pose &command) {
    if (!fault.empty() || selected.empty()) return false;
    center=rack;
    if (!engaged && target > geometry.contact && lift >= geometry.contact) {
      double wx=rack.x-base.x, wy=rack.y-base.y;
      if (std::abs(wx)>.04 || std::abs(wy)>.06 || std::abs(Wrap(rack.yaw))>.04) {
        fault="robot not aligned beneath selected rack"; return false;
      }
      if (geometry.platformTop+lift+geometry.tolerance < rack.z+geometry.payloadBottom) return false;
      double c=std::cos(base.yaw),s=std::sin(base.yaw);
      dx=c*wx+s*wy;dy=-s*wx+c*wy;dyaw=Wrap(rack.yaw-base.yaw);
      engaged=true;
    }
    if (!engaged) return false;
    double c=std::cos(base.yaw),s=std::sin(base.yaw);
    command={base.x+c*dx-s*dy,base.y+s*dx+c*dy,
      std::max(0.,geometry.platformTop+lift-geometry.payloadBottom),Wrap(base.yaw+dyaw)};
    releasing=(target==geometry.down && lift<=geometry.contact);
    if (releasing) command.z=0.;
    return true;
  }
  void Observe(double sim, const Pose &base, double lift, double liftVelocity, const Pose &rack) {
    if (!fault.empty()) { settled=false; carrying=false; return; }
    const bool jointStopped=std::abs(lift-target)<=geometry.tolerance && std::abs(liftVelocity)<.01;
    if (!jointStopped) stableSince=-1.;
    else if (stableSince<0.) stableSince=sim;
    settled=jointStopped && sim-stableSince>=geometry.settle;
    if (selected.empty()) { carrying=false; return; }
    center=rack;
    if (releasing && std::abs(rack.z)<=.002) { engaged=false; releasing=false; }
    const double expected=std::max(0.,geometry.platformTop+lift-geometry.payloadBottom);
    carrying=engaged && target==geometry.up && settled && expected>.15 &&
      std::abs(rack.z-expected)<=.002 && std::hypot(rack.x-base.x,rack.y-base.y)<.08;
    if (target==geometry.down && (engaged || releasing || std::abs(rack.z)>.002)) settled=false;
  }
};

struct Command { double vx{},vy{},wz{}; };
inline Command PlanarCommand(Command body,double yaw,bool allowRotation,bool engaged) {
  if (!std::isfinite(body.vx) || !std::isfinite(body.vy) || !std::isfinite(body.wz))
    throw std::runtime_error("nonfinite chassis command");
  if (std::hypot(body.vx,body.vy)>.31 || std::abs(body.wz)>.45)
    throw std::runtime_error("chassis speed limit exceeded");
  if (std::abs(body.wz)>.0001 && (!allowRotation || engaged || std::hypot(body.vx,body.vy)>.0001))
    throw std::runtime_error("rotation command rejected");
  double c=std::cos(yaw),s=std::sin(yaw);
  Command world{c*body.vx-s*body.vy,s*body.vx+c*body.vy,body.wz};
  if (std::abs(world.vx)>.002 && std::abs(world.vy)>.002)
    throw std::runtime_error("diagonal translation rejected");
  if (std::abs(world.vx)>=std::abs(world.vy)) world.vy=0.; else world.vx=0.;
  return world;
}
}
