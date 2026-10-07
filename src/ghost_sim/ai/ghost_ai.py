"""Model intent -> local XYZ resolution/validation -> existing 3D navigator."""
import argparse,json,math,os,subprocess,sys,time
from pathlib import Path
import xml.etree.ElementTree as ET
from ghost_sim.ai.ai_client import call_model,capture_camera
from ghost_sim.navigation.planner3d import Planner
from ghost_sim.paths import WORLD, CONFIG, SCRIPTS, REPORTS
STOP={'停止','停下','悬停','取消','取消导航','stop','cancel'}


def landmarks():
    result={'home':{'description':'房间中心，默认高度1.2米','position':[0.,0.,1.2]}}
    world=ET.parse(WORLD).find('world')
    for name,label,direction in [('green_box','绿色箱子',-1),('orange_box','橙色箱子',1)]:
        model=world.find(f"model[@name='{name}']")
        x,y,z,*_=map(float,model.findtext('pose').split())
        sx,sy,sz=map(float,model.findtext('link/collision/geometry/box/size').split())
        result[name]={'description':label+'旁边','position':[x+direction*(sx/2+.65),y,1.2]}
        result[name+'_above']={'description':label+'正上方','position':[x,y,z+sz/2+.65]}
    return result


def system_prompt(places):
    return ('你是可以在XYZ三轴自由飞行的球形幽灵的目标解析器，只输出一个JSON对象。'
            '位置单位米；z是球心离地高度，绝不是朝向。yaw另用弧度。'
            '合法动作：'
            '{"action":"navigate","position":[x,y,z]}，可另加"yaw":弧度；'
            '{"action":"navigate","target":"地标ID"}，可另加"z":绝对高度和"yaw":朝向；'
            '{"action":"relative","delta":[dx,dy,dz]}表示相对执行时位置的房间坐标位移；'
            '{"action":"height","z":绝对高度}只改变当前高度；'
            '{"action":"stop"}；{"action":"answer","text":"回答或澄清问题"}。'
            '例如升高半米用relative delta=[0,0,0.5]，降到1米用height z=1。'
            '去箱子上方选对应_above地标，不能去箱子实体中心。'
            '不指定朝向时不要编造yaw，本地保留当前朝向。'
            '只有明确移动请求才生成移动动作，模糊目标、多步骤任务、未登记物体应询问。'
            '前后左右未说明是房间轴还是朝向时应询问，不猜坐标；不能输出代码或速度。'
            '不能宣称已经到达，未附图不能声称看到物体。登记地标来自已知地图：'
            +json.dumps(places,ensure_ascii=False))


def parse(content):
    if not isinstance(content,str):raise ValueError('模型未返回文本')
    text=content.strip()
    if text.startswith('```') and text.endswith('```'):text='\n'.join(text.splitlines()[1:-1])
    try:data=json.loads(text)
    except ValueError:raise ValueError('模型没有返回合法JSON；不执行') from None
    if not isinstance(data,dict):raise ValueError('模型必须返回一个对象')
    a=data.get('action');keys=set(data)
    if a=='stop' and keys=={'action'}:return data
    if a=='answer' and keys=={'action','text'} and isinstance(data['text'],str):return data
    if a=='navigate':
        if 'position' in data and keys<={'action','position','yaw'}:return data
        if 'target' in data and keys<={'action','target','z','yaw'}:return data
    if a=='relative' and keys=={'action','delta'}:return data
    if a=='height' and keys=={'action','z'}:return data
    raise ValueError('不支持该动作或字段；不执行')


def finite(value):
    if type(value) not in (int,float) or not math.isfinite(value):raise ValueError('坐标与角度必须是有限数字')
    return float(value)


def vector(value):
    if not isinstance(value,list) or len(value)!=3:raise ValueError('必须提供三个XYZ数字')
    return [finite(v) for v in value]


def resolve(action,current,places):
    kind=action['action'];yaw=finite(current[3])
    if kind=='relative':position=[a+b for a,b in zip(current[:3],vector(action['delta']))]
    elif kind=='height':position=[current[0],current[1],finite(action['z'])]
    elif 'target' in action:
        target=action['target']
        if not isinstance(target,str) or target not in places:raise ValueError('未知地标')
        position=vector(places[target]['position'])
        if 'z' in action:position[2]=finite(action['z'])
    else:position=vector(action['position'])
    if 'yaw' in action:yaw=finite(action['yaw'])
    return {'action':'navigate','position':position,'yaw':math.atan2(math.sin(yaw),math.cos(yaw))}


def current_pose():
    import rclpy
    from tf2_msgs.msg import TFMessage
    rclpy.init();node=rclpy.create_node('ghost_ai_pose');poses=[]
    def receive(msg):
        for item in msg.transforms:
            if item.child_frame_id=='ghost':
                p,q=item.transform.translation,item.transform.rotation
                poses.append([p.x,p.y,p.z,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))])
    subscription=node.create_subscription(TFMessage,'/simulation/ground_truth/poses',receive,1)
    try:
        deadline=time.monotonic()+8
        while not poses and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
        if not poses:raise ValueError('未收到幽灵实时位姿，请启动域43的仿真')
        return poses[-1]
    finally:node.destroy_node();rclpy.try_shutdown()


def main():
    parser=argparse.ArgumentParser(description='自然语言三维导航；视觉模式仅问答')
    parser.add_argument('command');parser.add_argument('--dry-run',action='store_true')
    parser.add_argument('--vision',action='store_true');args=parser.parse_args()
    try:
        if args.command.strip().lower() in STOP:action={'action':'stop'}
        else:
            if not os.environ.get('STEPFUN_API_KEY','').strip():raise ValueError('缺少STEPFUN_API_KEY，请在本终端设置密钥')
            config=json.loads((CONFIG/'ghost_ai_config.json').read_text());places=landmarks()
            config['system_prompt']=system_prompt(places)
            image=capture_camera() if args.vision else None
            action=parse(call_model(args.command,config,image))
        if args.vision and action['action'] not in ('answer','stop'):raise ValueError('视觉模式只允许问答，请另发导航指令')
        if action['action'] in ('answer','stop'):
            print(json.dumps(action,ensure_ascii=False),flush=True)
            if action['action']=='stop' and not args.dry_run:
                return subprocess.run([str(SCRIPTS/'cancel_navigation.sh')],check=False).returncode
            return 0
        # Resolve relative motion only after the model responds, against a fresh physical pose.
        current=current_pose();intent=action;action=resolve(action,current,places)
        route=Planner().plan(current[:3],action['position'])
        record={'request':args.command,'intent':intent,'resolved':action,'reference_pose':current,
                'planned_waypoints':len(route),'dry_run':args.dry_run}
        (REPORTS/'ai_goal.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(action,ensure_ascii=False),flush=True)
        if args.dry_run:return 0
        command=[str(SCRIPTS/'navigate.sh'),*map(str,action['position']),'--yaw',str(action['yaw'])]
        return subprocess.run(command,check=False).returncode
    except KeyboardInterrupt:
        print('\n已中断；可用 ghost_ai.sh 停下 取消当前导航。');return 130
    except (ValueError,OSError) as error:
        print('未执行：'+str(error),file=sys.stderr);return 1

if __name__=='__main__':raise SystemExit(main())
