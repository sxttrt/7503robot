#include "actuator_core.hh"
#include <iostream>
#include <limits>
using namespace rack_simulation;
void Check(bool value,const char *message) { if (!value) throw std::runtime_error(message); }
int main() {
  try {
    for (const auto &name:{"rack_a","rack_b","rack_c","rack_d"}) {
      Payload p;Pose base{1.,1.,0.,3.141592653589793},rack{1.,1.,0.,0.},command;
      p.Select(name);p.Target(.3);
      Check(!p.Follow(base,.02,rack,command),"must wait until physical contact height");
      Check(p.Follow(base,.06,rack,command),"pickup must engage");
      rack=command;
      p.Observe(.1,base,.06,.2,rack);Check(!p.carrying,"moving joint is not carrying completion");
      p.Follow(base,.3,rack,command);rack=command;
      p.Observe(1.,base,.3,0.,rack);p.Observe(1.5,base,.3,0.,rack);
      Check(p.carrying && p.settled,"measured endpoint must confirm pickup");
      base={.25,1.05,0.,base.yaw};p.Follow(base,.3,rack,command);rack=command;
      Check(std::hypot(rack.x-base.x,rack.y-base.y)<.001,"rack must follow planar base");
      p.Target(0.);p.Follow(base,.03,rack,command);
      Check(p.engaged && !p.carrying && command.z==0.,"release requested but not yet measured");
      // A transport ACK cannot release a shelf whose measured height is still high.
      p.Observe(2.,base,0.,0.,rack);p.Observe(2.5,base,0.,0.,rack);
      Check(p.engaged && !p.settled,"unapplied ground target must not certify release");
      rack=command;p.Observe(2.6,base,0.,0.,rack);
      Check(!p.engaged && p.settled && !p.carrying,"measured ground pose completes lowering");
      p.Select("");Check(p.selected.empty() && p.fault.empty(),"selection must clear after release");
    }
    Payload wrong;wrong.Select("rack_b");wrong.Target(.3);Pose target;
    Check(!wrong.Follow({1.,1.,0.,3.141592653589793},.3,{1.2,1.,0.,0.},target) && !wrong.fault.empty(),"off-center pickup must fail");
    Payload busy;busy.Select("rack_a");busy.engaged=true;busy.Select("rack_b");Check(!busy.fault.empty(),"cannot switch engaged shelf");
    Payload unknown;unknown.Select("rack_unknown");Check(!unknown.fault.empty(),"reject unknown selection");
    Payload moving;moving.Select("rack_a");moving.Target(.3);moving.engaged=true;
    moving.Observe(1.,{1.,1.,0.,3.141592653589793},.3,.02,{1.,1.,.2625,0.});
    moving.Observe(2.,{1.,1.,0.,3.141592653589793},.3,.02,{1.,1.,.2625,0.});Check(!moving.settled,"endpoint alone cannot confirm moving lift");
    auto c=PlanarCommand({-.3,0.,0.},3.141592653589793,true,false);Check(c.vx>.299 && c.vy==0.,"body velocity transforms to fixed world axes");
    int rejected=0;
    for (auto command:{Command{.2,.2,0.},Command{0.,0.,.375},Command{std::numeric_limits<double>::quiet_NaN(),0.,0.}}) {
      try {PlanarCommand(command,0.,true,true);} catch (const std::exception &) {++rejected;}
    }
    Check(rejected==3,"diagonal, loaded rotation, and NaN commands must fail");
    std::cout<<"Payload pickup/transport/lowering for A B C D, command limits and failure semantics passed.\n";
  } catch (const std::exception &e) {std::cerr<<e.what()<<"\n";return 1;}
}
