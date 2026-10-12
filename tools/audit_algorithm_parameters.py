from pathlib import Path
import ast,re,json,csv,collections,xml.etree.ElementTree as ET
ROOT=Path('/home/wuta/WUTA2026_FINAL'); SRC=ROOT/'WUTA-FSD/ros2_ws/src'; OUT=ROOT/'docs'
packages={p.parent:ET.parse(p).getroot().findtext('name') for p in SRC.rglob('package.xml') if not any(x in p.parts for x in ['test','tests','build','install'])}
rows={}; dynamic=[]; launches=[]
def owner(f):
 ps=[p for p in packages if p in f.parents];return max(ps,key=lambda p:len(p.parts)) if ps else None
def add(pkg,name,default,f,line,kind='源码声明'):
 k=(pkg,name);r=rows.setdefault(k,{'package':pkg,'parameter':name,'declarations':[],'configs':[],'effective':None,'effective_source':'','status':''})
 r['declarations'].append({'default_expression':default,'file':str(f),'line':line,'kind':kind})
def splitargs(s):
 out=[];start=0;depth=0;quote=None;esc=False
 for i,c in enumerate(s):
  if quote:
   if esc:esc=False
   elif c=='\\':esc=True
   elif c==quote:quote=None
  elif c in "\"'":quote=c
  elif c in '([{':depth+=1
  elif c in ')]}':depth-=1
  elif c==',' and depth==0:out.append(s[start:i].strip());start=i+1
 out.append(s[start:].strip());return out
for f in SRC.rglob('*'):
 if f.suffix not in ['.cpp','.hpp','.h','.py'] or any(x in f.parts for x in ['test','tests','launch','examples','eval']):continue
 p=owner(f)
 if not p:continue
 pkg=packages[p];s=f.read_text(errors='replace')
 if f.suffix=='.py':
  try:t=ast.parse(s)
  except SyntaxError:continue
  for n in ast.walk(t):
   if isinstance(n,ast.Assign) and any(isinstance(a,ast.Name) and a.id=='defaults' for a in n.targets) and isinstance(n.value,ast.Dict):
    for k,v in zip(n.value.keys,n.value.values):
     try:add(pkg,ast.literal_eval(k),repr(ast.literal_eval(v)),f,k.lineno,'Python 默认字典')
     except (ValueError,TypeError):pass
   if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='declare_parameter' and n.args:
    if isinstance(n.args[0],ast.Constant):add(pkg,n.args[0].value,ast.unparse(n.args[1]) if len(n.args)>1 else '无默认值',f,n.lineno)
  continue
 for m in re.finditer(r'\bdeclare_parameter(?:\s*<[^()]*?>)?\s*\(',s):
  end=m.end();depth=1;quote=None;esc=False
  while end<len(s) and depth:
   c=s[end]
   if quote:
    if esc:esc=False
    elif c=='\\':esc=True
    elif c==quote:quote=None
   elif c in "\"'":quote=c
   elif c=='(':depth+=1
   elif c==')':depth-=1
   end+=1
  args=splitargs(s[m.end():end-1]);name=args[0];default=re.sub(r'\s+',' ',args[1]) if len(args)>1 else '无默认值';line=s[:m.start()].count('\n')+1
  if re.fullmatch(r'"[^"]+"',name):add(pkg,name[1:-1],default,f,line)
  else:dynamic.append({'package':pkg,'name_expression':name,'default_expression':default,'file':str(f),'line':line})
# Expand custom declaration helpers into actual concrete parameter names.
for d in dynamic:
 pkg=d['package'];e=d['name_expression'];f=Path(d['file']);line=d['line'];default=d['default_expression']
 if pkg=='controller' and e.startswith('ns + '):
  suffix=re.search(r'"([^"]+)"',e).group(1)
  if any(part in e for part in ('.pid.', '.throttle.', '.brake.')):prefixes=['acceleration','ebs','skidpad','trackdrive']
  elif '.finish.' in e:prefixes=['acceleration','ebs','skidpad']
  else:prefixes=[x+'.pure_pursuit' for x in ['acceleration','ebs','skidpad','trackdrive']]
  for pre in prefixes:add(pkg,pre+suffix,default,f,line,'动态名称展开')
 elif pkg=='path_generator' and e.startswith('prefix + '):
  suffix=re.search(r'"([^"]+)"',e).group(1)
  for pre in ['skidpad.geometry.crossing']+[m+'.anchor.'+a for m in ['skidpad','acceleration','ebs'] for a in ['entry','brake','stop']]:add(pkg,pre+suffix,default,f,line,'动态名称展开')
 elif pkg=='detection_fusion' and e.startswith('parameter_prefix + '):
  suffix=re.search(r'"([^"]+)"',e).group(1)
  for pre in ['debug_red','debug_orange']:add(pkg,pre+suffix,default,f,line,'按调试模式条件声明')
 elif pkg=='robot_localization':
  names={'odom_topic_name':'odomN','pose_topic_name':'poseN','twist_topic_name':'twistN','imu_topic_name':'imuN'}
  for var,pre in names.items():
   if var in e:
    tail=''.join(re.findall(r'"([^"]*)"',e));add(pkg,pre+tail,default,f,line,'传感器编号模板，N=0,1,…');break
# Additional RL dynamic names assembled outside declare_parameter.
rf=SRC/'localization/robot_localization/src/ros_filter.cpp';rs=rf.read_text()
for name in ['odomN_config','poseN_config','twistN_config','imuN_config','imuN_twist_rejection_threshold']:
 add('robot_localization',name,'长度15的布尔数组；声明/读取位置见 loadUpdateConfig',rf,rs[:rs.find('loadUpdateConfig')].count('\n')+1,'传感器配置数组模板')
def flatten(x,prefix=''):
 out={}
 for k,v in x.items():
  key=prefix+'.'+str(k) if prefix else str(k)
  if isinstance(v,dict):out.update(flatten(v,key))
  else:out[key]=v
 return out
config_only=[];plain=[]
for f in sorted(SRC.rglob('*.yaml'))+sorted((ROOT/'config').glob('*.yaml')):
 if f.is_symlink():continue
 if 'config' not in f.parts and f.parent.name!='params':continue
 if any(x in f.parts for x in ['test','tests']):continue
 try:obj=__import__('yaml').safe_load(f.read_text())
 except Exception:continue
 if not isinstance(obj,dict):continue
 p=owner(f);pkg=packages[p] if p else '独立控制标定'
 if f.name=='kiss_icp_hesai128.yaml':pkg='kiss_icp'
 if f==SRC/'localization/localization_manager/config/ekf.yaml':pkg='robot_localization'
 if f.name in ['zed_hardware.yaml','rsm1_hardware.yaml']:
  plain.append(str(f));continue
 ros=False
 for node,body in obj.items():
  if not isinstance(body,dict) or 'ros__parameters' not in body:continue
  ros=True
  for name,value in flatten(body['ros__parameters']).items():
   target=[pkg]
   if pkg=='独立控制标定':target=[pp for pp in ['controller','can_interface'] if (pp,name) in rows]
   for pp in target:
    key=(pp,name);cfg={'file':str(f),'node':node,'value':value}
    if key not in rows and pp=='robot_localization':
     template=next((k for k in rows if k[0]==pp and 'N' in k[1] and re.fullmatch(re.escape(k[1]).replace('N',r'\d+'),name)),None)
     if template:
      source=rows[template]['declarations'][0];add(pp,name,source['default_expression'],Path(source['file']),source['line'],'传感器编号模板实例')
    if key in rows:rows[key]['configs'].append(cfg)
    else:config_only.append({'package':pp,'parameter':name,**cfg})
 if not ros:plain.append(str(f))
# Source defaults retain expressions if not safely reducible.
def literal(s):
 s=s.strip().replace('true','True').replace('false','False')
 if s.startswith('std::string('):s=s[12:-1]
 try:return ast.literal_eval(s)
 except Exception:return '源码表达式：'+s
active_packages={'controller','can_interface','path_generator','boundary_detector','cone_map_builder','lidar_detection','camera_detection','detection_fusion','mission_manager','robot_localization'}
for r in rows.values():
 r['effective']=literal(r['declarations'][0]['default_expression']);r['effective_source']='源码默认值/表达式'
 r['status']='当前全栈启用' if r['package'] in active_packages else '当前全栈未启用'
 if r['package']=='camera_detection' and all('lwdetr' in d['file'] for d in r['declarations']):r['status']='LW-DETR 备选后端，当前未启用'
 if r['package']=='robot_localization' and not any('ros_filter' in d['file'] for d in r['declarations']):r['status']='定位库备选节点，当前未启用'
 if r['package']=='detection_fusion' and r['parameter'].startswith('debug_'):r['status']='仅对应调试模式生效；当前关闭'
 if r['parameter']=='model_path' and r['package']=='lidar_detection':r['status']='仅 dl 分支声明；该后端未实现'
for r in rows.values():
 if isinstance(r['effective'],str) and r['effective'].startswith('源码表达式：'):
  expr=r['declarations'][0]['default_expression'];member=expr.split('.')[-1]
  if re.fullmatch(r'[A-Za-z_]\w*',member):
   pool='\n'.join(f.read_text(errors='replace') for p in packages if packages[p]==r['package'] for f in p.rglob('*') if f.suffix in ['.hpp','.h'] and 'test' not in f.parts)
   vals=re.findall(r'\b'+re.escape(member)+r'\s*\{([^{}]+)\}',pool)
   if len(set(vals))==1:
    val=literal(vals[0])
    if not (isinstance(val,str) and val.startswith('源码表达式：')):
     r['effective']=val;r['effective_source']='头文件成员默认值；'+r['declarations'][0]['file']
basefiles={'controller':'control/controller/config/controller.yaml','can_interface':'system/can_interface/config/can_interface.yaml','path_generator':'planning/path_generator/config/path_generator.yaml','boundary_detector':'planning/boundary_detector/config/boundary_detector.yaml','cone_map_builder':'mapping/cone_map_builder/config/cone_map_builder.yaml','lidar_detection':'perception/lidar_detection/config/lidar_detection.yaml','detection_fusion':'perception/detection_fusion/config/fusion.yaml','mission_manager':'system/mission_manager/config/mission_manager.yaml','robot_localization':'localization/localization_manager/config/ekf.yaml','kiss_icp':'localization/kiss_icp_wrapper/config/kiss_icp_hesai128.yaml'}
for r in rows.values():
 for c in r['configs']:
  if c['file']==str(SRC/basefiles.get(r['package'],'-')):
   r['effective']=c['value'];r['effective_source']=c['file']
# Read the exact hardware launch argument defaults and parameter override dictionaries.
lf=SRC/'perception/detection_fusion/launch/fusion_hardware.launch.py';lt=ast.parse(lf.read_text());ld={}
for n in ast.walk(lt):
 if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='DeclareLaunchArgument' and n.args and isinstance(n.args[0],ast.Constant):
  opts={k.arg:k.value for k in n.keywords};val=opts.get('default_value');name=n.args[0].value
  default=ast.literal_eval(val) if isinstance(val,ast.Constant) else '(启动时传入)'
  ld[name]=default;launches.append({'name':name,'default':default,'file':str(lf),'line':n.lineno})
ld.update(model_input_width='1280',model_input_height='768',publish_annotated_image='true',start_map_builder='false',zed_publish_tf='false',zed_publish_map_tf='false',launch_rviz='false',model_path='启动变量 MODEL',calibration_path='启动变量 CALIBRATION')
def ev(n):
 if isinstance(n,ast.Constant):return n.value
 if isinstance(n,ast.Name):return {'confidence':float(ld['confidence_threshold']),'threads':int(ld['inference_threads']),'wait':float(ld['fusion_wait_sec']),'model':ld['model_path']}.get(n.id,'表达式：'+ast.unparse(n))
 if isinstance(n,ast.Call) and isinstance(n.func,ast.Name):
  name=n.func.id
  if name=='value':return ld.get(ast.literal_eval(n.args[0]),'?')
  if name in ['float','int','str']:return {'float':float,'int':int,'str':str}[name](ev(n.args[0]))
 if isinstance(n,ast.Compare) and len(n.ops)==1 and isinstance(n.ops[0],ast.Eq):return ev(n.left)==ev(n.comparators[0])
 return '表达式：'+ast.unparse(n)
for n in ast.walk(lt):
 if not(isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='Node'):continue
 kw={k.arg:k.value for k in n.keywords};pk=kw.get('package')
 if not isinstance(pk,ast.Constant):continue
 pkg=pk.value
 if pkg not in ['camera_detection','lidar_detection','detection_fusion']:continue
 executable=ast.unparse(kw.get('executable'));is_adapter='stereo' in executable
 for sub in ast.walk(kw.get('parameters',ast.List(elts=[],ctx=ast.Load()))):
  if not isinstance(sub,ast.Dict):continue
  for key,val in zip(sub.keys,sub.values):
   if not isinstance(key,ast.Constant):continue
   r=rows.get((pkg,key.value))
   if not r:continue
   if pkg=='camera_detection':
    # Distinguish detector/adapter instances that share topic parameter names.
    refs=[d for d in r['declarations'] if ('adapter' in d['file'])==is_adapter]
    if not refs:continue
    if key.value in ['output_topic','image_topic']:r.setdefault('instance_overrides',[]).append({'instance':'立体适配器' if is_adapter else '图像检测器','value':ev(val)})
   try:r['effective']=ev(val)
   except Exception:r['effective']='表达式：'+ast.unparse(val)
   r['effective_source']=str(lf)+':'+str(n.lineno)
# Overrides from the root shell for mission manager.
for name,value in {'lidar_topic':'/rslidar_points','imu_topic':'/chcnav/odometry','camera_topic':'/zed/zed_node/rgb/image_rect_color','use_ndt_race_localization':False}.items():
 r=rows.get(('mission_manager',name))
 if r:r['effective']=value;r['effective_source']='start_full_fsd.sh 命令行覆盖'
for r in rows.values():
 if r['package']=='camera_detection' and r['parameter'] in ['output_topic','depth_topic','info_topic','boxes_topic','output_info_topic']:
  vals={}
  for d in r['declarations']:
   file=d['file']
   if 'lwdetr' in file:continue
   node='stereo_detection_adapter_cpp' if 'adapter_cpp' in file else ('stereo_detection_adapter（备用）' if 'adapter.py' in file else 'yolov8_node')
   val=literal(d['default_expression'])
   if r['parameter']=='depth_topic':val=ld['depth_topic']
   if r['parameter']=='info_topic':val=ld['info_topic']
   vals[node]=val
  r['effective']=vals
# All explicit launch arguments, including optional simulation/localization paths.
for f in SRC.rglob('*.launch.py'):
 if f==lf or any(x in f.parts for x in ['test','tests']):continue
 try:tree=ast.parse(f.read_text())
 except SyntaxError:continue
 for n in ast.walk(tree):
  if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='DeclareLaunchArgument' and n.args and isinstance(n.args[0],ast.Constant):
   opts={k.arg:k.value for k in n.keywords};val=opts.get('default_value');default=ast.literal_eval(val) if isinstance(val,ast.Constant) else '(表达式或启动时传入)'
   launches.append({'name':n.args[0].value,'default':default,'file':str(f),'line':n.lineno})
for r in rows.values():
 if r['package']=='robot_localization' and r['parameter'].startswith(('pose','twist','imu')) and not any(c['file']==str(SRC/'localization/localization_manager/config/ekf.yaml') for c in r['configs']):r['status']='传感器编号模板/备选输入，当前 EKF 未配置'
 if r['package']=='lidar_detection' and r['parameter']=='ground_z_threshold':r['status']='阈值去地面或 RANSAC 失败回退时生效'
 if r['package']=='detection_fusion' and r['parameter']=='max_position_shift':r['status']='位置融合/对应关联分支使用；当前 fuse_positions=false'
# 当前实车入口的实际来源，以模块检查器的清单为准；旧launch只作为兼容入口。
from check_fsd_module_configs import manifest as module_manifest
active=module_manifest()
package_keys={'controller':'controller','can_interface':'can','lidar_detection':'lidar','detection_fusion':'fusion','cone_map_builder':'mapping','boundary_detector':'boundary','path_generator':'planning','mission_manager':'mission','robot_localization':'ekf'}
for item in rows.values():
 pkg=item['package'];key=package_keys.get(pkg)
 if key:
  current=flatten(active['parameters'][key])
  if item['parameter'] in current:
   item['effective']=current[item['parameter']];item['effective_source']=active['files'][key]
 if pkg=='camera_detection':
  name=item['parameter'];values={}
  for key,node in [('camera','yolov8_node'),('adapter','stereo_detection_adapter_cpp')]:
   current=flatten(active['parameters'][key])
   if name in current:values[node]=current[name]
  if values:
   item['effective']=next(iter(values.values())) if len(values)==1 else values
   item['effective_source']=active['files']['adapter'] if name in active['parameters']['adapter'] and name not in active['parameters']['camera'] else active['files']['camera']
  if name=='model_path':item['effective']=active['deployment']['model_path']

result={'scope':'源码静态审计；默认 start_full_fsd.sh 路径推导，未连接实时节点。源码表达式不冒充具体数值。','parameters':sorted(rows.values(),key=lambda r:(r['package'],r['parameter'])),'dynamic_declarations':dynamic,'config_without_matching_declaration':config_only,'plain_yaml_files':plain,'hardware_launch_arguments':launches,'package_roots':{v:str(k) for k,v in packages.items()}}
(OUT/'算法参数清单.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=True)+'\n')
with (OUT/'算法参数清单.csv').open('w',encoding='utf-8-sig',newline='') as f:
 w=csv.writer(f);w.writerow(['软件包','参数名','源码默认值表达式','配置中数值','全栈推导值','最终来源','生效条件','源码位置'])
 for r in result['parameters']:
  w.writerow([r['package'],r['parameter'],' / '.join(dict.fromkeys(d['default_expression'] for d in r['declarations'])),json.dumps(r['configs'],ensure_ascii=False),json.dumps(r['effective'],ensure_ascii=False),r['effective_source'],r['status'],'; '.join(d['file']+':'+str(d['line']) for d in r['declarations'])])
counts=collections.Counter(r['package'] for r in rows.values());print('各包参数名计数',dict(sorted(counts.items())));print('总计',len(rows),'动态声明',len(dynamic),'配置未匹配',len(config_only));print('非第三方配置未匹配',[(r['package'],r['parameter']) for r in config_only if r['package'] not in ['robot_localization','kiss_icp']])
