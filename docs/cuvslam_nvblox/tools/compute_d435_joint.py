#!/usr/bin/env python3
"""Turn Kalibr camera-IMU results into the URDF d435_joint (front_camera_link -> camera_link).

  python3 compute_d435_joint.py robot.urdf motion8-camchain-imucam.yaml motion9-camchain-imucam.yaml
  python3 compute_d435_joint.py --body lower_camera_body_link robot.urdf lower1-camchain-imucam.yaml   # lower_d435_joint

robot.urdf: the expanded URDF (xacro mk1_xsting.urdf.xacro > robot.urdf), for the CAD base -> imu_link and
base -> the D435 body link (--body, default front_camera_link for d435_joint; lower_camera_body_link for
lower_d435_joint). Each yaml is a Kalibr *-camchain-imucam.yaml (T_cam_imu of cam0 = left IR optical).

  T(base, camera_link) = T(base, imu_link) . T_ic . T(infra1_optical, camera_link)
  d435_joint           = T(base, front_camera_link)^-1 . T(base, camera_link)

T(infra1_optical, camera_link) is the RealSense driver's fixed rotation (camera_link sits on the left IR
depth origin, optical = z forward, x right, y down). With several results the translations are averaged and
the rotations averaged on SO(3); the spread between them is printed so a bad run stands out.
"""
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def R_to_rpy(R):
    p = np.arctan2(-R[2, 0], np.hypot(R[0, 0], R[1, 0]))
    return np.arctan2(R[2, 1], R[2, 2]), p, np.arctan2(R[1, 0], R[0, 0])


def T(xyz, rpy):
    M = np.eye(4)
    M[:3, :3] = rpy_to_R(*rpy)
    M[:3, 3] = xyz
    return M


def chain(urdf, link):
    """Pose of `link` in the URDF root, through fixed joints only (the path must not cross a moving joint)."""
    joints = {j.find('child').get('link'): j for j in ET.parse(urdf).getroot().findall('joint')}
    M = np.eye(4)
    while link in joints:
        j = joints[link]
        if j.get('type') not in ('fixed', 'floating'):
            sys.exit(f'{j.get("name")} on the path is {j.get("type")}; only fixed joints are expected')
        o = j.find('origin')
        xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
        rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
        M = T(xyz, rpy) @ M
        link = j.find('parent').get('link')
    return M


def mean_rotation(Rs):
    U, _, Vt = np.linalg.svd(sum(Rs))
    R = U @ Vt
    return R if np.linalg.det(R) > 0 else U @ np.diag([1, 1, -1]) @ Vt


def angle_deg(Ra, Rb):
    return np.degrees(np.arccos(np.clip((np.trace(Ra.T @ Rb) - 1) / 2, -1, 1)))


def main():
    args = sys.argv[1:]
    body = 'front_camera_link'
    if args[:1] == ['--body']:
        body, args = args[1], args[2:]
    if len(args) < 2:
        sys.exit(__doc__)
    urdf, results = args[0], args[1:]
    base_imu = chain(urdf, 'imu_link')
    base_front = chain(urdf, body)
    opt_link = np.linalg.inv(T([0, 0, 0], [-np.pi / 2, 0, -np.pi / 2]))  # camera_link -> optical, inverted

    joints = []
    for path in results:
        T_ci = np.array(yaml.safe_load(open(path))['cam0']['T_cam_imu'])
        joint = np.linalg.inv(base_front) @ base_imu @ np.linalg.inv(T_ci) @ opt_link
        joints.append(joint)
        cam_in_imu = np.linalg.inv(T_ci)[:3, 3]
        print(f'{path}\n  camera in imu_link (fwd/left/up) {np.round(cam_in_imu * 100, 2)} cm, '
              f'd435_joint xyz {np.round(joint[:3, 3], 5)} rpy {np.round(R_to_rpy(joint[:3, :3]), 5)}')

    xyz = np.mean([j[:3, 3] for j in joints], axis=0)
    R = mean_rotation([j[:3, :3] for j in joints])
    if len(joints) > 1:
        dt = max(np.linalg.norm(a[:3, 3] - b[:3, 3]) for a in joints for b in joints) * 100
        dr = max(angle_deg(a[:3, :3], b[:3, :3]) for a in joints for b in joints)
        print(f'spread between results: {dt:.2f} cm, {dr:.2f} deg  (README: within 2 cm and 1 deg)')
    r, p, y = R_to_rpy(R)
    print(f'\n<origin xyz="{xyz[0]:.5f} {xyz[1]:.5f} {xyz[2]:.5f}" rpy="{r:.5f} {p:.5f} {y:.5f}"/>')
    cam = base_front @ T(xyz, (r, p, y))
    print(f'camera_link in base_footprint: {np.round(cam[:3, 3], 4)} m')


if __name__ == '__main__':
    main()
