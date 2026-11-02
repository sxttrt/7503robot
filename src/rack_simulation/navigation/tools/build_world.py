#!/usr/bin/env python3
"""Generate the rack world and real QR textures from scripts/scene.py."""
import argparse
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import qrcode

ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT/'scripts'))
import scene


def add(parent,tag,text=None,**attrs):
    n=ET.SubElement(parent,tag,attrs)
    if text is not None:n.text=str(text)
    return n


def pose(parent,values):add(parent,'pose',' '.join(str(v) for v in values))


def inertia(link,mass,size):
    x,y,z=size
    inertial=add(link,'inertial');add(inertial,'mass',mass)
    tensor=add(inertial,'inertia')
    for name,val in dict(ixx=mass*(y*y+z*z)/12,iyy=mass*(x*x+z*z)/12,
                         izz=mass*(x*x+y*y)/12,ixy=0,ixz=0,iyz=0).items():add(tensor,name,val)


def box(link,name,size,position=(0,0,0,0,0,0),color='.35 .52 .72 1',mask='0xFFFF',collision=True):
    for kind in (('collision','visual') if collision else ('visual',)):
        n=add(link,kind,name=name+'_'+kind);pose(n,position)
        add(add(add(n,'geometry'),'box'),'size',' '.join(map(str,size)))
        if kind=='collision':add(add(add(n,'surface'),'contact'),'collide_bitmask',mask)
        else:
            m=add(n,'material');add(m,'ambient',color);add(m,'diffuse',color)


def link(model,name,size,mass,position=(0,0,0,0,0,0),color='.35 .52 .72 1',mask='0x0002'):
    l=add(model,'link',name=name);pose(l,position);add(l,'gravity','false')
    inertia(l,mass,size);box(l,name,size,color=color,mask=mask)
    return l


def fixed(model,parent,child):
    j=add(model,'joint',name=parent+'_'+child+'_fixed',type='fixed')
    add(j,'parent',parent);add(j,'child',child)


def pose_plugin(model,rate=30):
    p=add(model,'plugin',filename='gz-sim-pose-publisher-system',name='gz::sim::systems::PosePublisher')
    for k,v in dict(publish_model_pose='true',publish_link_pose='false',publish_collision_pose='false',
                    publish_nested_model_pose='false',update_frequency=rate,use_pose_vector_msg='true').items():add(p,k,v)


def rack(world,name,values,texture):
    x0,x1,y0,y1,sy,_=values;cx,cy=scene.rack_model_center(name)
    m=add(world,'model',name=f'rack_{name.lower()}');pose(m,(cx,cy,0,0,0,0));add(m,'static','true')
    w,l=x1-x0,y1-y0
    link(m,'top',(w,l,scene.TOP_T),1.,(0,0,scene.RACK_H-scene.TOP_T/2,0,0,0))
    for i,y in enumerate((y0+scene.LEG_T/2,y1-scene.LEG_T/2)):
        nm=f'leg_{i}'
        link(m,nm,(w*.9,scene.LEG_T,scene.PAYLOAD_BOTTOM),.2,
             (0,y-cy,scene.PAYLOAD_BOTTOM/2,0,0,0))
        fixed(m,'top',nm)
    qr=add(m,'link',name='qr');pose(qr,(x1-cx+.004,sy-cy,scene.QR_Z,0,0,0))
    add(qr,'gravity','false');inertia(qr,.02,(.002,scene.QR_SIZE,scene.QR_SIZE))
    visual=add(qr,'visual',name='qr_code');geom=add(visual,'geometry');plane=add(geom,'plane')
    add(plane,'normal','1 0 0');add(plane,'size',f'{scene.QR_SIZE} {scene.QR_SIZE}')
    material=add(visual,'material');add(material,'ambient','1 1 1 1');add(material,'diffuse','1 1 1 1')
    metal=add(add(material,'pbr'),'metal');add(metal,'albedo_map','../materials/'+texture.name)
    add(metal,'metalness',0);add(metal,'roughness',1)
    fixed(m,'top','qr')


def robot(world):
    m=add(world,'model',name='robot',canonical_link=scene.BASE_FRAME)
    x,y,h=scene.model_start_pose();pose(m,(x,y,0,0,0,h))
    # World -> X slide -> Y slide -> yaw -> base. Z/roll/pitch are absent DOFs:
    # physical lift reactions are supported by the world, not a floating base.
    for name in ('x_slide','y_slide'):
        carriage=add(m,'link',name=name);add(carriage,'gravity','false')
        inertia(carriage,.01,(.01,.01,.01))
    for name,parent,child,kind,direction,limits in (
        ('chassis_x','world','x_slide','prismatic','-1 0 0',(-x+scene.ROBOT_L/2,scene.FIELD_X-x-scene.ROBOT_L/2)),
        ('chassis_y','x_slide','y_slide','prismatic','0 -1 0',(-y+scene.ROBOT_W/2,scene.FIELD_Y-y-scene.ROBOT_W/2)),
        ('chassis_yaw','y_slide',scene.BASE_FRAME,'revolute','0 0 1',(-.06,3.141592653589793/6+.06)),
    ):
        j=add(m,'joint',name=name,type=kind);add(j,'parent',parent);add(j,'child',child)
        axis=add(j,'axis');add(axis,'xyz',direction,expressed_in='__model__')
        limit=add(axis,'limit');add(limit,'lower',limits[0]);add(limit,'upper',limits[1])
        add(limit,'effort',1000);add(limit,'velocity',1)
    # No pose service controls the robot. The plugin writes joint velocity each
    # physics step and has its own watchdog if the ROS forwarding process dies.
    actuator=add(m,'plugin',filename='librack_hybrid_actuator.so',name='rack_simulation::HybridActuator')
    for k,v in dict(allow_scan_rotation='true',field_x=scene.FIELD_X,field_y=scene.FIELD_Y,
        half_x=scene.ROBOT_L/2,half_y=scene.ROBOT_W/2,initial_yaw=h,
        platform_top=scene.PLATFORM_TOP,payload_bottom=scene.PAYLOAD_BOTTOM,
        lift_up=scene.LIFT_UP,lift_upper=scene.LIFT_UPPER,lift_tolerance=scene.LIFT_TOL).items():add(actuator,k,v)
    base=add(m,'link',name=scene.BASE_FRAME);add(base,'gravity','false')
    inertia(base,5.,(scene.ROBOT_L,scene.ROBOT_W,scene.ROBOT_H))
    box(base,'chassis',(scene.ROBOT_L,scene.ROBOT_W,scene.ROBOT_H),
        (0,0,scene.ROBOT_H/2+.005,0,0,0),'.35 .35 .4 1','0x0001')
    # MS200 housing, cable and adapter-board visuals remain inside the chassis envelope.
    box(base,'ms200_housing',(.045,.045,.025),(0,0,scene.LIDAR_Z-.0225,0,0,0),'.08 .08 .08 1',collision=False)
    box(base,'ms200_cable',(.055,.004,.004),(.035,-.024,.112,0,0,0),'.04 .04 .04 1',collision=False)
    box(base,'ms200_adapter',(.03,.022,.003),(.065,-.024,.108,0,0,0),'.1 .4 .15 1',collision=False)
    camera=add(base,'sensor',name='camera',type='camera');pose(camera,(*scene.CAM_POS,0,scene.CAM_PITCH,0))
    add(camera,'topic','/camera');add(camera,'update_rate',15);add(camera,'always_on',1)
    c=add(camera,'camera');add(c,'horizontal_fov',1.047)
    im=add(c,'image');add(im,'width',640);add(im,'height',480);add(im,'format','RGB_INT8')
    clip=add(c,'clip');add(clip,'near',.05);add(clip,'far',10)
    lidar=add(base,'sensor',name='ms200',type='gpu_lidar');pose(lidar,(0,0,scene.LIDAR_Z,0,0,0))
    add(lidar,'topic','/scan');add(lidar,'update_rate',10);add(lidar,'always_on',1)
    add(lidar,'visualize','false')
    ray=add(lidar,'ray');horizontal=add(add(ray,'scan'),'horizontal')
    for k,v in dict(samples=720,resolution=1,min_angle=-3.14159265,max_angle=3.14159265).items():add(horizontal,k,v)
    rng=add(ray,'range')
    for k,v in dict(min=.05,max=8.,resolution=.01).items():add(rng,k,v)
    noise=add(ray,'noise');add(noise,'type','gaussian');add(noise,'mean',0);add(noise,'stddev',.005)
    # Shelf is a kinematic static obstacle moved in the Gazebo simulation thread. Its bit 0x0002 must not contact
    # platform bit 0x0004, otherwise contact correction and pose sync fight each other.
    lift=link(m,'lift_platform',(scene.PLATFORM_L,scene.PLATFORM_W,scene.PLATFORM_T),scene.PLATFORM_MASS,
              (0,0,scene.PLATFORM_TOP-scene.PLATFORM_T/2,0,0,0),'.6 .6 .6 1','0x0004')
    joint=add(m,'joint',name='lift_joint',type='prismatic')
    add(joint,'parent',scene.BASE_FRAME);add(joint,'child','lift_platform')
    axis=add(joint,'axis');add(axis,'xyz','0 0 1');limit=add(axis,'limit')
    add(limit,'lower',scene.LIFT_DOWN);add(limit,'upper',scene.LIFT_UPPER);add(limit,'effort',100)
    p=add(m,'plugin',filename='gz-sim-joint-position-controller-system',name='gz::sim::systems::JointPositionController')
    for k,v in dict(joint_name='lift_joint',topic='/simulation/lift_setpoint',p_gain=scene.LIFT_P_GAIN,i_gain=0,d_gain=scene.LIFT_D_GAIN,
                    i_max=10,i_min=-10,initial_position=scene.LIFT_DOWN).items():add(p,k,v)
    add(m,'plugin',filename='gz-sim-joint-state-publisher-system',name='gz::sim::systems::JointStatePublisher')



def static_box(world,name,size,position,color,collision=True):
    model=add(world,'model',name=name);add(model,'static','true')
    l=add(model,'link',name='link');box(l,name,size,position,color,collision=collision)


def build(out):
    sdf=ET.Element('sdf',version='1.9');world=add(sdf,'world',name='rack_world');add(world,'gravity','0 0 -9.8')
    phys=add(world,'physics',name='physics',type='ignored');add(phys,'max_step_size',.002);add(phys,'real_time_factor',1)
    for filename,classname in [('physics','Physics'),('user-commands','UserCommands'),
                               ('scene-broadcaster','SceneBroadcaster'),('sensors','Sensors')]:
        add(world,'plugin',filename=f'gz-sim-{filename}-system',name=f'gz::sim::systems::{classname}')
    sun=add(world,'light',name='sun',type='directional');add(sun,'cast_shadows','false')
    pose(sun,(0,0,10,0,0,0));add(sun,'diffuse','.9 .9 .9 1');add(sun,'specular','.1 .1 .1 1');add(sun,'direction','0 0 -1')
    sc=add(world,'scene');add(sc,'ambient','.8 .8 .8 1');add(sc,'background','.7 .7 .7 1');add(sc,'grid','false')
    static_box(world,'ground',(scene.FIELD_X+.12,scene.FIELD_Y+.12,.02),
               (scene.FIELD_X/2,scene.FIELD_Y/2,-.01,0,0,0),'.75 .75 .72 1')
    for name,size,position in [
        ('wall_n',(scene.FIELD_X+.12,.06,.30),(scene.FIELD_X/2,scene.FIELD_Y+.03,.15,0,0,0)),
        ('wall_s',(scene.FIELD_X+.12,.06,.30),(scene.FIELD_X/2,-.03,.15,0,0,0)),
        ('wall_w',(.06,scene.FIELD_Y,.30),(-.03,scene.FIELD_Y/2,.15,0,0,0)),
        ('wall_e',(.06,scene.FIELD_Y,.30),(scene.FIELD_X+.03,scene.FIELD_Y/2,.15,0,0,0))]:
        static_box(world,name,size,position,'.65 .67 .72 1')
    for name,zone,color in [('start_zone',scene.START_ZONE,'.55 .55 .3 1'),('end_zone',scene.END_ZONE,'.3 .55 .35 1')]:
        x0,x1,y0,y1=zone
        static_box(world,name,(x1-x0,y1-y0,.002),((x0+x1)/2,(y0+y1)/2,.001,0,0,0),color,collision=False)
    for name,(x,y) in scene.DROP.items():
        static_box(world,f'drop_{name.lower()}',(.20,.20,.002),(x,y,.002,0,0,0),'1 1 1 1',collision=False)
    materials=ROOT/'materials';materials.mkdir(exist_ok=True)
    for name,values in scene.RACKS.items():
        texture=materials/f'qr_{name.lower()}.png'
        code=qrcode.QRCode(box_size=12,border=4);code.add_data(values[5]);code.make(fit=True)
        code.make_image(fill_color='black',back_color='white').convert('RGB').save(texture)
        rack(world,name,values,texture)
    robot(world)
    ET.indent(sdf,space='  ');out.parent.mkdir(parents=True,exist_ok=True)
    ET.ElementTree(sdf).write(out,encoding='utf-8',xml_declaration=True)
    print(f'world/QR generated from scene.py: {out}')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--out',type=Path,default=ROOT/'worlds/rack_world.sdf')
    parser.add_argument('--odom-source',choices=['encoder','wheel','pose'],default='pose',help='legacy compatibility argument; navigation odom is radar-only')
    args=parser.parse_args();build(args.out)


if __name__=='__main__':main()
