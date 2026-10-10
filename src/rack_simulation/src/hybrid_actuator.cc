// Gazebo-only actuation. Ground truth never enters the navigation pipeline.
#include "actuator_core.hh"
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <chrono>
#include <mutex>
#include <sstream>
#include <iomanip>
#include <map>

namespace rack_simulation {
using namespace gz::sim;
class HybridActuator : public System, public ISystemConfigure, public ISystemPreUpdate, public ISystemPostUpdate {
  gz::transport::Node node;
  Entity model{kNullEntity}, base{kNullEntity};
  std::array<Entity,4> joints{};
  std::map<std::string,Entity> racks;
  std::map<std::string,gz::transport::Node::Publisher> poses;
  gz::transport::Node::Publisher status;
  gz::transport::Node::Publisher liftSetpoint;
  Payload payload;
  Command command;
  bool allowRotation{true};
  double fieldX{3.02},fieldY{2.}, halfX{.12},halfY{.08},initialYaw{3.141592653589793};
  double lastPublish{-1.};
  double lastLiftPublish{-1.},publishedLift{-1.};
  std::mutex mutex;
  Command incoming;
  std::string selection;
  bool selectionChanged{false},liftChanged{false};
  double liftTarget{0.};
  std::chrono::steady_clock::time_point seen{};
  std::chrono::steady_clock::time_point liftSeen{};
  static Pose ReadPose(Entity entity,const EntityComponentManager &ecm) {
    auto p=worldPose(entity,ecm);return {p.X(),p.Y(),p.Z(),p.Rot().Yaw()};
  }
  static double ReadJoint(Entity e,const EntityComponentManager &ecm,bool velocity=false) {
    if (velocity) { auto v=ecm.Component<components::JointVelocity>(e);return v && !v->Data().empty()?v->Data()[0]:0.; }
    auto p=ecm.Component<components::JointPosition>(e);return p && !p->Data().empty()?p->Data()[0]:0.;
  }
  bool Feedback(const EntityComponentManager &ecm) const {
    if (base==kNullEntity || racks.size()!=4) return false;
    for (auto joint:joints) {
      auto p=ecm.Component<components::JointPosition>(joint);
      auto v=ecm.Component<components::JointVelocity>(joint);
      if (!p || !v || p->Data().empty() || v->Data().empty()) return false;
    }
    return true;
  }
  void PublishPose(const std::string &name,const Pose &p,const gz::msgs::Time &stamp) {
    gz::msgs::Pose_V message;*message.mutable_header()->mutable_stamp()=stamp;
    auto pose=message.add_pose();pose->set_name(name);
    gz::msgs::Set(pose,gz::math::Pose3d(p.x,p.y,p.z,0.,0.,p.yaw));poses.at(name).Publish(message);
  }
 public:
  void Configure(const Entity &entity,const std::shared_ptr<const sdf::Element> &sdf,
                 EntityComponentManager &,EventManager &) override {
    model=entity;
    if (sdf->HasElement("allow_scan_rotation")) allowRotation=sdf->Get<bool>("allow_scan_rotation");
    if (sdf->HasElement("field_x")) fieldX=sdf->Get<double>("field_x");
    if (sdf->HasElement("field_y")) fieldY=sdf->Get<double>("field_y");
    if (sdf->HasElement("half_x")) halfX=sdf->Get<double>("half_x");
    if (sdf->HasElement("half_y")) halfY=sdf->Get<double>("half_y");
    if (sdf->HasElement("initial_yaw")) initialYaw=sdf->Get<double>("initial_yaw");
    if (sdf->HasElement("platform_top")) payload.geometry.platformTop=sdf->Get<double>("platform_top");
    if (sdf->HasElement("payload_bottom")) payload.geometry.payloadBottom=sdf->Get<double>("payload_bottom");
    if (sdf->HasElement("lift_up")) payload.geometry.up=sdf->Get<double>("lift_up");
    if (sdf->HasElement("lift_upper")) payload.geometry.upper=sdf->Get<double>("lift_upper");
    if (sdf->HasElement("lift_tolerance")) payload.geometry.tolerance=sdf->Get<double>("lift_tolerance");
    payload.geometry.contact=payload.geometry.payloadBottom-payload.geometry.platformTop;
    status=node.Advertise<gz::msgs::StringMsg>("/simulation/actuator/status");
    liftSetpoint=node.Advertise<gz::msgs::Double>("/simulation/lift_setpoint");
    for (const auto &name:{"robot","rack_a","rack_b","rack_c","rack_d"})
      poses.emplace(name,node.Advertise<gz::msgs::Pose_V>(std::string("/model/")+name+"/pose"));
    node.Subscribe("/simulation/chassis_cmd",&HybridActuator::OnCommand,this);
    node.Subscribe("/simulation/payload_name",&HybridActuator::OnSelection,this);
    node.Subscribe("/simulation/lift_cmd",&HybridActuator::OnLift,this);
  }
  void OnCommand(const gz::msgs::Twist &m) {
    std::lock_guard<std::mutex> lock(mutex);
    incoming={m.linear().x(),m.linear().y(),m.angular().z()};seen=std::chrono::steady_clock::now();
  }
  void OnSelection(const gz::msgs::StringMsg &m) {
    std::lock_guard<std::mutex> lock(mutex);selection=m.data();selectionChanged=true;
  }
  void OnLift(const gz::msgs::Double &m) {
    std::lock_guard<std::mutex> lock(mutex);liftTarget=m.data();liftChanged=true;liftSeen=std::chrono::steady_clock::now();
  }
  void PreUpdate(const UpdateInfo &info,EntityComponentManager &ecm) override {
    Model robot(model);
    if (base==kNullEntity) {
      base=robot.LinkByName(ecm,"base_link");
      const std::array<std::string,4> names={"chassis_x","chassis_y","chassis_yaw","lift_joint"};
      for (size_t i=0;i<4;++i) joints[i]=robot.JointByName(ecm,names[i]);
      for (auto joint:joints) if (joint!=kNullEntity) {
        ecm.SetComponentData<components::JointPosition>(joint,{});
        ecm.SetComponentData<components::JointVelocity>(joint,{});
      }
      for (const auto &name:{"rack_a","rack_b","rack_c","rack_d"}) {
        auto e=ecm.EntityByComponents(components::Model(),components::Name(name));
        if (e!=kNullEntity) racks.emplace(name,e);
      }
    }
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (selectionChanged) { payload.Select(selection);selectionChanged=false; }
      if (liftChanged) { payload.Target(liftTarget);liftChanged=false; }
      command=(std::chrono::duration<double>(std::chrono::steady_clock::now()-seen).count()<.30)?incoming:Command{};
    }
    if (info.paused || !Feedback(ecm)) return;
    const double measuredLift=ReadJoint(joints[3],ecm);
    {
      std::lock_guard<std::mutex> lock(mutex);
      // Lost lift command heartbeat during motion holds the measured joint;
      // it never guesses success or automatically lowers carried cargo.
      if (std::abs(measuredLift-payload.target)>payload.geometry.tolerance &&
          std::chrono::duration<double>(std::chrono::steady_clock::now()-liftSeen).count()>.6) {
        if (payload.fault.empty()) payload.fault="lift command heartbeat expired during motion";
        payload.Target(std::clamp(measuredLift,0.,payload.geometry.upper));
      }
    }
    // Only validated targets reach the physical JointPositionController.
    const double sim=std::chrono::duration<double>(info.simTime).count();
    if (payload.target!=publishedLift || sim-lastLiftPublish>=.05) {
      gz::msgs::Double setpoint;setpoint.set_data(payload.target);liftSetpoint.Publish(setpoint);
      publishedLift=payload.target;lastLiftPublish=sim;
    }
    const Pose p=ReadPose(base,ecm);
    Command world;
    try {
      world=PlanarCommand(command,p.yaw,allowRotation,payload.engaged);
      const double dt=std::chrono::duration<double>(info.dt).count();
      if (p.x+world.vx*dt<halfX || p.x+world.vx*dt>fieldX-halfX ||
          p.y+world.vy*dt<halfY || p.y+world.vy*dt>fieldY-halfY) throw std::runtime_error("chassis outside field");
      const double angle=Wrap(p.yaw+world.wz*dt-initialYaw);
      if (angle<-.06 || angle>3.141592653589793/6+.06) throw std::runtime_error("scan rotation exceeded allowed envelope");
    } catch (const std::exception &e) { if (payload.fault.empty()) payload.fault=e.what(); }
    if (!payload.fault.empty()) world={};
    ecm.SetComponentData<components::JointVelocityCmd>(joints[0],{world.vx});
    ecm.SetComponentData<components::JointVelocityCmd>(joints[1],{world.vy});
    ecm.SetComponentData<components::JointVelocityCmd>(joints[2],{world.wz});
    if (!payload.selected.empty() && racks.count(payload.selected)) {
      auto rack=racks.at(payload.selected);Pose target;
      if (payload.Follow(p,ReadJoint(joints[3],ecm),ReadPose(rack,ecm),target))
        ecm.SetComponentData<components::WorldPoseCmd>(rack,gz::math::Pose3d(target.x,target.y,target.z,0.,0.,target.yaw));
    }
  }
  void PostUpdate(const UpdateInfo &info,const EntityComponentManager &ecm) override {
    if (info.paused || !Feedback(ecm)) return;
    double sim=std::chrono::duration<double>(info.simTime).count();
    Pose p=ReadPose(base,ecm);double lift=ReadJoint(joints[3],ecm),liftV=ReadJoint(joints[3],ecm,true);
    Pose rack=payload.selected.empty()?payload.center:ReadPose(racks.at(payload.selected),ecm);
    payload.Observe(sim,p,lift,liftV,rack);
    if (sim-lastPublish<1./30.) return;
    lastPublish=sim;gz::msgs::Time stamp;stamp.set_sec(static_cast<int64_t>(sim));stamp.set_nsec(static_cast<int32_t>((sim-std::floor(sim))*1e9));
    PublishPose("robot",p,stamp);for (auto &[name,e]:racks) PublishPose(name,ReadPose(e,ecm),stamp);
    const bool baseStopped=std::hypot(ReadJoint(joints[0],ecm,true),ReadJoint(joints[1],ecm,true))<.01 && std::abs(ReadJoint(joints[2],ecm,true))<.01;
    std::ostringstream s;s<<std::setprecision(15)<<std::boolalpha;
    s<<"{\"stamp\":{\"sec\":"<<stamp.sec()<<",\"nanosec\":"<<stamp.nsec()<<"},\"source\":\"gazebo_planar_actuator\",\"selected\":\""<<payload.selected
     <<"\",\"carrying\":"<<payload.carrying<<",\"engaged\":"<<payload.engaged<<",\"lift\":"<<lift<<",\"lift_target\":"<<payload.target
     <<",\"lift_settled\":"<<payload.settled<<",\"base_motion_stopped\":"<<baseStopped<<",\"lift_motion_stopped\":"<<(std::abs(liftV)<.01)
     <<",\"relative_xy\":["<<payload.dx<<","<<payload.dy<<"],\"payload_center\":["<<payload.center.x<<","<<payload.center.y<<","<<payload.center.z
     <<"],\"ready\":"<<payload.fault.empty()<<",\"fault\":\""<<payload.fault<<"\",\"base_pose\":["<<p.x<<","<<p.y<<","<<p.z<<","<<p.yaw<<"]}";
    gz::msgs::StringMsg message;message.set_data(s.str());status.Publish(message);
  }
};
}
GZ_ADD_PLUGIN(rack_simulation::HybridActuator,gz::sim::System,
  rack_simulation::HybridActuator::ISystemConfigure,
  rack_simulation::HybridActuator::ISystemPreUpdate,
  rack_simulation::HybridActuator::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(rack_simulation::HybridActuator,"rack_simulation::HybridActuator")
