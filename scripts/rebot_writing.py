"""One continuous 3-D printing-point path spelling 国庆快乐.

Hand-authored centerline glyphs; no fonts, extrusion, support material, or hardware.
Every connector is part of the same drawn path, offset behind the writing plane.
"""
import numpy as np
import mujoco as mj
import pinocchio as pin
from rebot_core import Robot, HOME, ROOT, solve_ik, IKParams

# Coordinates are normalized typographic coordinates: x right, y upward.
GLYPHS = {
 '国': [
  [( .10,.90),(.10,.10)],
  [(.10,.90),(.90,.90),(.90,.10)],
  [(.26,.72),(.74,.72)], [(.50,.72),(.50,.27)],
  [(.28,.51),(.72,.51)], [(.23,.27),(.77,.27)],
  [(.65,.44),(.72,.35)], [(.10,.10),(.90,.10)],
 ],
 '庆': [
  [(.51,.96),(.58,.86)], [(.18,.79),(.92,.79)],
  [(.20,.79),(.20,.48),(.17,.27),(.08,.09)],
  [(.29,.52),(.87,.52)],
  [(.59,.70),(.56,.45),(.49,.29),(.36,.15),(.23,.09)],
  [(.56,.45),(.67,.29),(.78,.15),(.92,.08)],
 ],
 '快': [
  [(.14,.69),(.08,.48)], [(.28,.72),(.35,.58)],
  [(.23,.94),(.23,.08)],
  [(.42,.73),(.79,.73),(.79,.46)],
  [(.35,.46),(.92,.46)],
  [(.58,.93),(.58,.56),(.55,.34),(.44,.17),(.32,.08)],
  [(.58,.46),(.70,.26),(.82,.13),(.94,.08)],
 ],
 '乐': [
  [(.80,.92),(.60,.85),(.28,.78)],
  [(.28,.78),(.23,.51),(.88,.51)],
  [(.55,.70),(.55,.13),(.49,.08),(.41,.15)],
  [(.35,.36),(.23,.21),(.10,.13)],
  [(.69,.36),(.82,.22),(.92,.12)],
 ],
}
CENTER = np.array([.38, 0., .45])
RIGHT = np.array([0., 1., 0.])
UP = np.array([-.5, 0., np.sqrt(3)/2])
NORMAL = np.cross(RIGHT, UP)
ROTATION = np.column_stack((NORMAL, RIGHT, np.cross(NORMAL, RIGHT)))
SIZE = .060
GAP = .012
DEPTH = .012
PALETTE = [(.30,.95,1.,1.), (.99,.77,.20,1.), (.45,1.,.54,1.), (1.,.42,.57,1.), (.31,.24,.43,.70)]


def connected_walks(strokes):
    """Trace touching strokes as a graph, revisiting ink instead of adding bridges.

    Split lines at every intersection. Each component is walked continuously;
    only truly disconnected components need an out-of-plane connector.
    """
    segments=[(np.array(a,float),np.array(b,float)) for stroke in strokes for a,b in zip(stroke,stroke[1:])]
    cuts=[[0.,1.] for _ in segments]
    def cross(a,b):return a[0]*b[1]-a[1]*b[0]
    for i,(a,b) in enumerate(segments):
        for j,(c,d) in enumerate(segments[:i]):
            v=b-a;w=d-c;den=cross(v,w)
            if abs(den)>1e-10:
                t=cross(c-a,w)/den;u=cross(c-a,v)/den
                if -1e-8<=t<=1+1e-8 and -1e-8<=u<=1+1e-8:
                    cuts[i].append(float(np.clip(t,0,1)));cuts[j].append(float(np.clip(u,0,1)))
    adjacency={};edges=[]
    for (a,b),ts in zip(segments,cuts):
        ts=sorted(set(round(t,8) for t in ts))
        for u,v in zip(ts,ts[1:]):
            x=tuple(np.round(a+(b-a)*u,7));y=tuple(np.round(a+(b-a)*v,7))
            if x==y:continue
            edge=len(edges);edges.append((x,y))
            adjacency.setdefault(x,[]).append((y,edge));adjacency.setdefault(y,[]).append((x,edge))
    visited=set();walks=[];last=None
    while len(visited)<len(edges):
        candidates=[x for x,neighbors in adjacency.items() if any(e not in visited for _,e in neighbors)]
        root=candidates[0] if last is None else min(candidates,key=lambda x:np.linalg.norm(np.array(x)-last))
        todo=[root];component_nodes={root};component_edges=set()
        while todo:
            node=todo.pop()
            for neighbor,e in adjacency[node]:
                component_edges.add(e)
                if neighbor not in component_nodes:component_nodes.add(neighbor);todo.append(neighbor)
        walk=[root]
        def visit(node):
            for neighbor,e in adjacency[node]:
                if e in visited:continue
                visited.add(e);walk.append(neighbor);visit(neighbor)
                if component_edges<=visited:return
                walk.append(node)
        visit(root);walks.append(walk);last=np.array(walk[-1])
    return walks


def writing_program(speed=.045):
    points=[];styles=[]
    total=4*SIZE+3*GAP
    for char_index,(char,strokes) in enumerate(GLYPHS.items()):
        for stroke in connected_walks(strokes):
            mapped=[CENTER+RIGHT*(-total/2+char_index*(SIZE+GAP)+x*SIZE)+UP*((y-.5)*SIZE) for x,y in stroke]
            if not points:
                points.append(mapped[0])
            elif np.linalg.norm(points[-1]-mapped[0])>1e-8:
                # A real continuous bridge in space. No pen-up or hidden jump.
                for p in [points[-1]-NORMAL*DEPTH,mapped[0]-NORMAL*DEPTH,mapped[0]]:
                    if np.linalg.norm(p-points[-1])>1e-8:
                        points.append(p);styles.append(4)
            for p in mapped[1:]:
                points.append(p);styles.append(char_index)
    first=points[0]
    document=dict(version=1,name='一笔三维书写：国庆快乐',steps=[
        dict(op='gripper',name='夹持虚拟打印头姿态',finger_m=.016,seconds=1),
        dict(op='move_tcp',name='接近书写起点',xyz=(first-NORMAL*.025).tolist(),orientation=ROTATION.tolist(),seconds=3),
        dict(op='move_tcp',name='定位打印点',xyz=first.tolist(),orientation=ROTATION.tolist(),seconds=1),
        dict(op='wait',name='起点稳定',seconds=.4),
        dict(op='trace_path',name='连续一笔：国庆快乐',points=np.array(points).tolist(),
             styles=styles,orientation=ROTATION.tolist(),speed_m_s=speed),
        dict(op='move_tcp',name='书写完成，移开打印头',xyz=(points[-1]-NORMAL*.04).tolist(),orientation=ROTATION.tolist(),seconds=2),
    ])
    return document


def orientation(value, planner, seed):
    if isinstance(value,str):
        if value=='keep':return planner.fk(seed).rotation
        if value=='down':return np.array([[0,0,1],[0,1,0],[-1,0,0]],float)
        raise ValueError('orientation 支持 keep、down 或 3×3 旋转矩阵')
    rotation=np.asarray(value,dtype=float)
    if rotation.shape!=(3,3) or not np.all(np.isfinite(rotation)) or not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(rotation),1,atol=1e-6):
        raise ValueError('orientation 必须为合法的 3×3 旋转矩阵')
    return rotation


def compile_trace(raw,planner,seed):
    points=np.asarray(raw['points'],float)
    if points.ndim!=2 or points.shape[1]!=3 or not 2<=len(points)<=400 or not np.all(np.isfinite(points)):
        raise ValueError('trace_path 需要 2–400 个三维有限坐标点')
    styles=raw.get('styles',[0]*(len(points)-1))
    if not isinstance(styles,list) or len(styles)!=len(points)-1 or any(type(s) is not int or not 0<=s<5 for s in styles):
        raise ValueError('styles 需要与每段路径一一对应的 0–4 颜色编号')
    speed=float(raw.get('speed_m_s',.045))
    if not np.isfinite(speed) or not .005<=speed<=.08:
        raise ValueError('书写速度必须为 0.005–0.08 m/s')
    R=orientation(raw.get('orientation','keep'),planner,seed)
    if np.linalg.norm(planner.fk(seed).translation-points[0])>.0002:
        raise ValueError('请先移动到 trace_path 的起点')
    q=seed.copy();times=[0.];qs=[q.copy()];xyz=[points[0].copy()];edge_ids=[0];clock=0.
    # Densely solve each straight Cartesian edge. Smoothstep brings velocity to
    # zero at corners without lifting or discontinuing the deposited path.
    for edge,(a,b) in enumerate(zip(points,points[1:])):
        length=float(np.linalg.norm(b-a))
        if length<1e-7:raise ValueError('路径包含零长度线段')
        duration=max(.10,1.5*length/speed)
        count=max(3,int(np.ceil(1.5*length/.0015)),int(np.ceil(duration/.05)))
        local=[]
        for u in np.linspace(0,1,count+1)[1:]:
            p=a+(b-a)*(u*u*(3-2*u))
            solved=solve_ik(planner.pm,planner.pm.createData(),planner.fid,pin.SE3(R,p),q,IKParams(tolerance=1e-6),controlled_joints=6)
            if not solved.success:raise ValueError(f'书写线段 {edge+1} 无逆解')
            next_q=solved.q.copy();planner.command(next_q)
            if np.max(np.abs(next_q-q))>.08:raise ValueError('书写路径出现逆解跳变')
            if planner.contacts(next_q,.016):raise ValueError(f'书写线段 {edge+1} 发生模型自碰撞')
            local.append((u,next_q,p));q=next_q
        velocities=np.diff(np.vstack([qs[-1]]+[item[1] for item in local]),axis=0)/(duration/count)
        duration*=max(1.,float(np.max(np.abs(velocities)))/.6)
        for u,q,p in local:
            times.append(clock+u*duration);qs.append(q);xyz.append(p);edge_ids.append(edge)
        clock+=duration
    if clock>600:raise ValueError('单条书写路径过长')
    return dict(times=np.array(times),qs=np.array(qs),xyz=np.array(xyz),edge_ids=np.array(edge_ids),
                points=points,styles=styles,seconds=clock,target=q.copy())


class Trail:
    def __init__(self):
        self.clear()

    def clear(self):
        self.points=[];self.styles=[];self.enabled=False;self.length=0.

    def sample(self,point,style,force=False):
        point=np.asarray(point,float).copy()
        if not self.points:
            self.points.append(point);return
        distance=float(np.linalg.norm(point-self.points[-1]))
        if distance<1e-8 or (distance<.0008 and not force):return
        if len(self.points)>=6500:raise RuntimeError('轨迹点数超出可视化容量')
        self.length+=distance;self.points.append(point);self.styles.append(int(style))

    def render(self,scene,robot,ghost_robot=False):
        if ghost_robot and self.enabled:
            for g in scene.geoms[:scene.ngeom]:
                if g.objtype==mj.mjtObj.mjOBJ_GEOM and g.objid>=0 and robot.m.geom_bodyid[g.objid] in robot.robot_bodies:
                    g.rgba[:]=[.22,.29,.35,.16]

        def geometry(kind,rgba,start,end=None,width=.001):
            if scene.ngeom>=scene.maxgeom:raise RuntimeError('MuJoCo 可视化几何容量不足')
            g=scene.geoms[scene.ngeom]
            mj.mjv_initGeom(g,kind,np.array([width]*3),np.asarray(start),np.eye(3).ravel(),np.array(rgba,np.float32))
            if end is not None:mj.mjv_connector(g,kind,width,np.asarray(start),np.asarray(end))
            g.emission=.08 if rgba==PALETTE[4] else .45;g.specular=0.
            scene.ngeom+=1
        for a,b,style in zip(self.points,self.points[1:],self.styles):
            geometry(mj.mjtGeom.mjGEOM_CAPSULE,PALETTE[style],a,b,width=.00038 if style==4 else .0011)
        if self.enabled:
            pose=robot.fk(robot.q);tip=pose.translation
            geometry(mj.mjtGeom.mjGEOM_CAPSULE,(.82,.87,.94,1),tip-pose.rotation[:,0]*.038,tip,width=.0024)
            geometry(mj.mjtGeom.mjGEOM_SPHERE,(1.,1.,.85,1),tip,width=.003)


def print_camera():
    c=mj.MjvCamera();c.lookat[:]=[.32,0,.39];c.distance=.85;c.azimuth=180;c.elevation=-25
    return c
