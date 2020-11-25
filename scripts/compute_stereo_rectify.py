import cv2
import numpy as np
import yaml

calib_file = '/home/chutsu/kalibr_ws/euroc_calib/camchain.yaml'

calib_yaml = open(calib_file, 'r')
calib = yaml.safe_load(calib_yaml)


def form_K(intrinsics):
    fx = intrinsics[0]
    fy = intrinsics[1]
    cx = intrinsics[2]
    cy = intrinsics[3]
    return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

# cam0_K = form_K(calib['cam0']['intrinsics'])
# cam0_dist = np.array(calib['cam0']['distortion'] + [0.0])
# cam1_K = form_K(calib['cam1']['intrinsics'])
# cam1_dist = np.array(calib['cam1']['distortion'] + [0.0])
# T_SC0 = np.reshape(np.array(calib['T_imu0_cam0']['data']), (4, 4))
# T_SC1 = np.reshape(np.array(calib['T_imu0_cam1']['data']), (4, 4))

cam0_K = form_K(calib['cam0']['intrinsics'])
cam0_dist = np.array(calib['cam0']['distortion_coeffs'] + [0.0])
cam1_K = form_K(calib['cam1']['intrinsics'])
cam1_dist = np.array(calib['cam1']['distortion_coeffs'] + [0.0])
T_C0S = np.reshape(np.array(calib['cam0']['T_cam_imu']), (4, 4))
T_C1S = np.reshape(np.array(calib['cam1']['T_cam_imu']), (4, 4))
T_SC0 = np.linalg.inv(T_C0S)
T_SC1 = np.linalg.inv(T_C1S)

T_C0S = np.linalg.inv(T_SC0)
T_C0C1 = np.dot(T_C0S, T_SC1)
T_C1C0 = np.linalg.inv(T_C0C1)

image_size = (752, 480)
R = T_C1C0[0:3, 0:3]
t = T_C1C0[0:3, 3]

retval = cv2.stereoRectify(cam0_K, cam0_dist,
                           cam1_K, cam1_dist,
                           image_size, R, t,
                           alpha=0)

R0 = retval[0]
R1 = retval[1]
P0 = retval[2]
P1 = retval[3]

P_fx = P0[0, 0]
P_fy = P0[1, 1]
P_cx = P0[0, 2]
P_cy = P0[1, 2]
baseline = -1 * P1[0, 3]


ORBSLAM3_CALIB='''\
%YAML:1.0

#--------------------------------------------------------------------------------------------
# Camera Parameters. Adjust them!
#--------------------------------------------------------------------------------------------
Camera.type: "PinHole"

# Camera calibration and distortion parameters (OpenCV) (equal for both cameras after stereo rectification)
# euroc
Camera.fx: {P_fx}
Camera.fy: {P_fy}
Camera.cx: {P_cx}
Camera.cy: {P_cy}

Camera.k1: 0.0
Camera.k2: 0.0
Camera.p1: 0.0
Camera.p2: 0.0

Camera.width: 752
Camera.height: 480

# Camera frames per second
Camera.fps: 20.0

# stereo baseline times fx
Camera.bf: {baseline}

# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
Camera.RGB: 1

# Close/Far threshold. Baseline times.
ThDepth: 35.0 # 35

# Transformation from camera 0 to body-frame (imu)
Tbc: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: {T_SC0}

# IMU noise
IMU.NoiseGyro: 1.7e-04 # 1.6968e-04
IMU.NoiseAcc: 2.0e-03 # 2.0000e-3
IMU.GyroWalk: 1.9393e-05
IMU.AccWalk: 3.e-03 # 3.0000e-3
IMU.Frequency: 200

#--------------------------------------------------------------------------------------------
# Stereo Rectification. Only if you need to pre-rectify the images.
# Camera.fx, .fy, etc must be the same as in LEFT.P
#--------------------------------------------------------------------------------------------

LEFT.height: 480
LEFT.width: 752
LEFT.D: !!opencv-matrix
   rows: 1
   cols: 5
   dt: d
   data: {cam0_dist}

LEFT.K: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: {cam0_K}

LEFT.R:  !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: {R0}

LEFT.Rf:  !!opencv-matrix
   rows: 3
   cols: 3
   dt: f
   data: {R0}

LEFT.P:  !!opencv-matrix
   rows: 3
   cols: 4
   dt: d
   data: {P0}

RIGHT.height: 480
RIGHT.width: 752
RIGHT.D: !!opencv-matrix
   rows: 1
   cols: 5
   dt: d
   data: {cam1_dist}

RIGHT.K: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: {cam1_K}

RIGHT.R:  !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: {R1}

RIGHT.P:  !!opencv-matrix
   rows: 3
   cols: 4
   dt: d
   data: {P1}

#--------------------------------------------------------------------------------------------
# ORB Parameters
#--------------------------------------------------------------------------------------------

# ORB Extractor: Number of features per image
ORBextractor.nFeatures: 1200

# ORB Extractor: Scale factor between levels in the scale pyramid
ORBextractor.scaleFactor: 1.2

# ORB Extractor: Number of levels in the scale pyramid
ORBextractor.nLevels: 8

# ORB Extractor: Fast threshold
# Image is divided in a grid. At each cell FAST are extracted imposing a minimum response.
# Firstly we impose iniThFAST. If no corners are detected we impose a lower value minThFAST
# You can lower these values if your images have low contrast
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

#--------------------------------------------------------------------------------------------
# Viewer Parameters
#--------------------------------------------------------------------------------------------
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3
Viewer.ViewpointX: 0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500
'''.format(
    P_fx=P_fx,
    P_fy=P_fy,
    P_cx=P_cx,
    P_cy=P_cy,
    T_SC0=T_SC0.flatten().tolist(),
    cam0_K=cam0_K.flatten().tolist(),
    cam1_K=cam1_K.flatten().tolist(),
    cam0_dist=cam0_dist.flatten().tolist(),
    cam1_dist=cam1_dist.flatten().tolist(),
    R0=R0.flatten().tolist(), R1=R1.flatten().tolist(),
    P0=P0.flatten().tolist(), P1=P1.flatten().tolist(),
    baseline=baseline
)
print(ORBSLAM3_CALIB)
