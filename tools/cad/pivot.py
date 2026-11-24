import json,math,sys
sys.path.insert(0,sys.argv[2])
from steptree import mul,inv
t=json.load(open(sys.argv[1]))
st=t['stow']
def find(prefix,elev):
    for i,e in enumerate(st):
        if e['leaf'] and e['path'].split('/')[1].startswith(prefix):
            yield i
claw=next(find('Assembly 1',0))
carr=[i for i in find('Superstructure',0) if 'Pivot' in st[i]['path']][0]
stage1=[i for i,e in enumerate(st) if e['leaf'] and 'Elevator' in e['path'] and abs(t['score'][i]['M'][1][3]-e['M'][1][3]+0.536)<0.002][0]
print('carriage part',st[carr]['path'][:120]); print('stage1 part',st[stage1]['path'][:120])
for pose in ['score','intake']:
    Rc=mul(t[pose][claw]['M'],inv(st[claw]['M']))   # claw world motion
    Tc=mul(t[pose][carr]['M'],inv(st[carr]['M']))   # carriage motion (pure translation)
    A=mul(inv(Tc),Rc)  # claw motion in carriage frame (stow-referenced)
    # fixed point in YZ: (I-R) p = t
    a,b,c,d=1-A[1][1],-A[1][2],-A[2][1],1-A[2][2]
    det=a*d-b*c
    py=( d*A[1][3]-b*A[2][3])/det; pz=(-c*A[1][3]+a*A[2][3])/det
    ang=math.degrees(math.atan2(A[2][1],A[1][1]))
    print(pose,'carriage dY,dZ',round(Tc[1][3],4),round(Tc[2][3],4),'travel',round(math.hypot(Tc[1][3],Tc[2][3]),4),
          'arm rot about +X deg',round(ang,2),'pivot (stow, cad) y,z',round(py,4),round(pz,4))
# Pivot assembly extents (cad stow) to sanity check
