import sys, numpy as np, cv2, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, Image
p=sys.argv[1]; out=sys.argv[2]
r=rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=p,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/opencr/imu_corrected','/camera/camera/infra1/image_rect_raw']))
imu=[]; img_t=[]; det=[]; i=0; snaps=[]
while r.has_next():
    t,d,_=r.read_next()
    if t=='/opencr/imu_corrected':
        m=deserialize_message(d,Imu)
        imu.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9, np.linalg.norm([m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z]), np.linalg.norm([m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z])])
    else:
        m=deserialize_message(d,Image); ts=m.header.stamp.sec+m.header.stamp.nanosec*1e-9; img_t.append(ts)
        if i%5==0:
            g=np.frombuffer(m.data,np.uint8).reshape(m.height,m.width)
            ok,_=cv2.findChessboardCorners(g,(9,6),cv2.CALIB_CB_ADAPTIVE_THRESH|cv2.CALIB_CB_NORMALIZE_IMAGE)
            det.append((ts,ok))
            if i%600==0: snaps.append(cv2.resize(g,(320,240)))
        i+=1
imu=np.array(imu); img_t=np.array(img_t); det=np.array(det)
t0=min(imu[0,0],img_t[0])
di=np.diff(imu[:,0]); dc=np.diff(img_t)
print(f'IMU n={len(imu)} rate={1/np.median(di):.1f}Hz dt max {di.max()*1e3:.0f}ms, gaps>40ms {(di>0.04).sum()}, non-increasing {(di<=0).sum()}')
print(f'IMG n={len(img_t)} rate={1/np.median(dc):.1f}Hz dt max {dc.max()*1e3:.0f}ms, drops(>50ms) {(dc>0.05).sum()}')
print(f'overlap: img {img_t[0]-t0:.1f}~{img_t[-1]-t0:.1f}s, imu {imu[0,0]-t0:.1f}~{imu[-1,0]-t0:.1f}s')
print(f'board detected (OpenCV 9x6, every 5th frame): {det[:,1].mean()*100:.0f}%  ({int(det[:,1].sum())}/{len(det)})')
print('  sec   gyro|w|mean  max   |acc|min-max   board%')
for s in range(0,int(img_t[-1]-t0)+1,10):
    a=imu[(imu[:,0]-t0>=s)&(imu[:,0]-t0<s+10)]; b=det[(det[:,0]-t0>=s)&(det[:,0]-t0<s+10)]
    if len(a)==0: continue
    print(f'{s:4d}-{s+10:<4d} {a[:,1].mean():.2f}  {a[:,1].max():.2f}   {a[:,2].min():5.1f}-{a[:,2].max():5.1f}   {b[:,1].mean()*100 if len(b) else 0:4.0f}%  '+'#'*int(a[:,1].mean()*30))
rows=[np.hstack(snaps[k:k+4]+[np.zeros_like(snaps[0])]*(4-len(snaps[k:k+4]))) for k in range(0,len(snaps),4)]
cv2.imwrite(out,np.vstack(rows))
