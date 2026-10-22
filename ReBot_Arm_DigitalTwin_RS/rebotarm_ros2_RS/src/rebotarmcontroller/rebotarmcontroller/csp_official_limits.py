"""Published RS values; control mode and continuous/peak ratings stay explicit.

Seeed Web guide: 1.5 rad/s, 4 rad/s², 30 rad/s³.
MotorBridge rebot-arm-robstride template: 36/14 N·m, 57/16 A.
Seeed motor specifications: continuous rated torque RS06 11, RS00 5 N·m.
The template current ceilings are peak operating limits, not continuous ratings.
"""
SPEED_MAX = 1.5
ACCELERATION_MAX = 4.0
JERK_MAX = 30.0
DRIVE_TORQUE = (36.,36.,36.,14.,14.,14.)
DRIVE_CURRENT = (57.,57.,57.,16.,16.,16.)
RATED_TORQUE = (11.,11.,11.,5.,5.,5.)
SOURCES = {
    'motion':'https://wiki.seeedstudio.com/cn/rebot_arm_b601_rs_web_simulator_developer_guide/',
    'drive':'https://github.com/tianrking/motorbridge-studio/blob/main/src/lib/robotArm.js',
    'ratings':'https://wiki.seeedstudio.com/rebot_arm_b601_rs_mit_control/',
}
