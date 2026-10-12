"""前向规划窗口的几何距离：首段允许向后最多5米，其余段不外推。"""
import math
def path_distance(x,y,pts,back_extension_m=5.):
    distances=[]
    for i,((ax,ay),(bx,by)) in enumerate(zip(pts,pts[1:])):
        vx,vy=bx-ax,by-ay;den=vx*vx+vy*vy
        if den<1e-9:continue
        t=((x-ax)*vx+(y-ay)*vy)/den
        t=max(-back_extension_m/math.sqrt(den) if i==0 else 0,min(1,t))
        distances.append(math.hypot(x-ax-t*vx,y-ay-t*vy))
    return min(distances) if distances else None
if __name__=='__main__':
    assert path_distance(0,0,[(3,0),(4,0)])==0
    assert path_distance(0,1,[(3,0),(4,0)])==1
    assert path_distance(-10,0,[(3,0),(4,0)])==8
    print('前向窗口几何测试通过')
