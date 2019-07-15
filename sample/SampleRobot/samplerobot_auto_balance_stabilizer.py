#!/usr/bin/env python

try:
    from hrpsys.hrpsys_config import *
    import OpenHRP
except:
    print "import without hrpsys"
    import rtm
    from rtm import *
    from OpenHRP import *
    import waitInput
    from waitInput import *
    import socket
    import time

import math
from subprocess import check_output
from distutils.version import StrictVersion

def getRTCListWithABS():
    return [
        ['seq', "SequencePlayer"],
        ['sh', "StateHolder"],
        ['fk', "ForwardKinematics"],
        ['tf', "TorqueFilter"],
        ['kf', "KalmanFilter"],
        ['vs', "VirtualForceSensor"],
        ['rmfo', "RemoveForceSensorLinkOffset"],
        ['octd', "ObjectContactTurnaroundDetector"],
        ['es', "EmergencyStopper"],
        ['rfu', "ReferenceForceUpdater"],
        ['ic', "ImpedanceController"],
        # ['abc', "AutoBalancer"],
        # ['st', "Stabilizer"],
        ['abst', "AutoBalanceStabilizer"],
        ['co', "CollisionDetector"],
        ['tc', "TorqueController"],
        ['te', "ThermoEstimator"],
        ['hes', "EmergencyStopper"],
        ['el', "SoftErrorLimiter"],
        ['tl', "ThermoLimiter"],
        ['bp', "Beeper"],
        ['acf', "AccelerationFilter"],
        ['log', "DataLogger"]
    ]

def defJointGroups():
    rleg_6dof_group = ['rleg', ['RLEG_HIP_R', 'RLEG_HIP_P', 'RLEG_HIP_Y', 'RLEG_KNEE', 'RLEG_ANKLE_P', 'RLEG_ANKLE_R']]
    lleg_6dof_group = ['lleg', ['LLEG_HIP_R', 'LLEG_HIP_P', 'LLEG_HIP_Y', 'LLEG_KNEE', 'LLEG_ANKLE_P', 'LLEG_ANKLE_R']]
    torso_group = ['torso', ['WAIST_P', 'WAIST_R', 'CHEST']]
    head_group = ['head', []]
    rarm_group = ['rarm', ['RARM_SHOULDER_P', 'RARM_SHOULDER_R', 'RARM_SHOULDER_Y', 'RARM_ELBOW', 'RARM_WRIST_Y', 'RARM_WRIST_P', 'RARM_WRIST_R']]
    larm_group = ['larm', ['LARM_SHOULDER_P', 'LARM_SHOULDER_R', 'LARM_SHOULDER_Y', 'LARM_ELBOW', 'LARM_WRIST_Y', 'LARM_WRIST_P', 'LARM_WRIST_R']]
    return [rleg_6dof_group, lleg_6dof_group, torso_group, head_group, rarm_group, larm_group]

def initHcf():
    global hcf, initial_pose, arm_front_pose, half_sitting_pose, root_rot_x_pose, root_rot_y_pose, pose_list, hrpsys_version, four_legs_mode_pose, autobalancer_limbs
    hcf = HrpsysConfigurator()
    hcf.getRTCList = getRTCListWithABS

    hcf.init ("SampleRobot(Robot)0", "$(PROJECT_DIR)/../model/sample1.wrl")
    hcf.connectLoggerPort(hcf.abst, 'baseRpyOut') # Just for checking
    hcf.Groups = defJointGroups()

    # set initial pose from sample/controller/SampleController/etc/Sample.pos
    initial_pose = [-7.779e-005,  -0.378613,  -0.000209793,  0.832038,  -0.452564,  0.000244781,  0.31129,  -0.159481,  -0.115399,  -0.636277,  0,  0,  0.637045,  -7.77902e-005,  -0.378613,  -0.000209794,  0.832038,  -0.452564,  0.000244781,  0.31129,  0.159481,  0.115399,  -0.636277,  0,  0,  -0.637045,  0,  0,  0]
    arm_front_pose = [-7.778932e-05,-0.378613,-0.00021,0.832039,-0.452564,0.000245,-1.5708,-0.159481,-0.115399,-0.349066,0.0,0.0,0.0,-7.778932e-05,-0.378613,-0.00021,0.832039,-0.452564,0.000245,-1.5708,0.159481,0.115399,-0.349066,0.0,0.0,0.0,0.0,0.0,0.0]
    half_sitting_pose = [-0.000158,-0.570987,-0.000232,1.26437,-0.692521,0.000277,0.31129,-0.159481,-0.115399,-0.636277,0.0,0.0,0.0,-0.000158,-0.570987,-0.000232,1.26437,-0.692521,0.000277,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.0,0.0,0.0]
    root_rot_x_pose = [-0.241557,-0.634167,0.011778,1.30139,-0.668753,0.074236,0.31129,-0.159481,-0.115399,-0.636277,0.0,0.0,0.0,-0.233491,-0.555191,0.011181,1.13468,-0.580942,0.065086,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.0,0.0,0.0]
    root_rot_y_pose = [8.251963e-05,-0.980029,-0.000384,1.02994,-0.398115,-0.000111,0.31129,-0.159481,-0.115399,-0.636277,0.0,0.0,0.0,8.252625e-05,-0.980033,-0.000384,1.02986,-0.398027,-0.000111,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.0,0.0,0.0]
    four_legs_mode_pose = [0.0, -0.349066, 0.0, 0.820305, -0.471239, 0.0, 0.493231, 0.008013, 0.000304, -1.608, 0.008019, -0.456023, 0.637045, 0.0, -0.349066, 0.0, 0.820305, -0.471239, 0.0, 0.493231, -0.008013, -0.000304, -1.608, -0.008019, -0.456023, -0.637045, 0.0, 0.0, 0.0]
    pose_list=[half_sitting_pose, root_rot_x_pose, root_rot_y_pose, arm_front_pose, four_legs_mode_pose]

    autobalancer_limbs = ["rleg", "lleg", "rarm", "larm"]

    hrpsys_version = hcf.seq.ref.get_component_profile().version
    print("hrpsys_version = %s" % hrpsys_version)

def initAutoBalancer():
    # on < 315.5.0 this outputs huge error log message
    hcf.seq_svc.setJointAngles(initial_pose, 2.0)
    hcf.seq_svc.waitInterpolation()

def testPoseList(pose_list, initial_pose):
    for pose in pose_list:
        hcf.seq_svc.setJointAngles(pose, 1.0)
        hcf.waitInterpolation()
        hcf.seq_svc.setJointAngles(initial_pose, 1.0)
        hcf.waitInterpolation()

def saveLogForCheckParameter(log_fname="/tmp/test-samplerobot-auto-balance-stabilizer-check-param"):
    hcf.setMaxLogLength(1)
    hcf.clearLog()
    time.sleep(0.1)
    hcf.saveLog(log_fname)

def checkParameterFromLog(port_name, log_fname="/tmp/test-samplerobot-auto-balance-stabilizer-check-param",
                          save_log=True, rtc_name="SampleRobot(Robot)0"):
    if save_log:
        saveLogForCheckParameter(log_fname)
    return map(float, open(log_fname + "." + rtc_name + "_" + port_name, "r").readline().split(" ")[1:-1])

def checkActualBaseAttitude(ref_rpy = None, thre=0.1): # degree
    '''Check whether the robot falls down based on actual robot base-link attitude.
    '''
    act_rpy = checkParameterFromLog("WAIST")[3:]
    print("hogehoge")
    if ref_rpy == None:
        print("hogehoge None")
        ref_rpy = checkParameterFromLog("baseRpyOut", rtc_name="sh", save_log=False)
    print(len(act_rpy))
    print(len(ref_rpy))
    ret = abs(math.degrees(act_rpy[0]-ref_rpy[0])) < thre and abs(math.degrees(act_rpy[1]-ref_rpy[1])) < thre
    print >> sys.stderr, "  ret = ", ret, ", actual base rpy = (", act_rpy, "), ", "reference base rpy = (", ref_rpy, ")"
    assert (ret)
    return ret

def Quaternion2Angle(q):
    w, v = q[0], q[1:]
    theta = math.acos(w) * 2.0
    return theta

def Quaternion2RotMatrixZ(q):
    theta = Quaternion2Angle(q)
    return numpy.array([[numpy.cos(theta), -numpy.sin(theta), 0],
                        [numpy.sin(theta),  numpy.cos(theta), 0],
                        [               0,                 0, 1]])


def calcDiffFootMidCoords (prev_dst_foot_midcoords):
    '''Calculate difference from previous dst_foot_midcoords and current dst_foot_midcoords.
    Returns difx, dify, difth, which are gopos parameters
    '''
    new_dst_foot_midcoords=hcf.abst_svc.getFootstepParam()[1].dst_foot_midcoords
    # Check diff
    difxy = (Quaternion2RotMatrixZ(prev_dst_foot_midcoords.rot).transpose()).dot((numpy.array([new_dst_foot_midcoords.pos])-numpy.array([prev_dst_foot_midcoords.pos])).transpose())
    difth = math.degrees(Quaternion2Angle(new_dst_foot_midcoords.rot)-Quaternion2Angle(prev_dst_foot_midcoords.rot))
    return [difxy[0,0], difxy[1,0], difth]

def checkGoPosParam (goalx, goaly, goalth, prev_dst_foot_midcoords):
    '''Check whether goPos argument are correctly achieved based on dst_foot_midcoords values.
    goPos params should be "new_dst_foot_midcoords - prev_dst_foot_midcoords"
    '''
    # Check diff
    [difx, dify, difth] = calcDiffFootMidCoords(prev_dst_foot_midcoords)
    ret = (abs(difx-goalx) < 5e-5 and abs(dify-goaly) < 5e-5 and abs(difth-goalth) < 1e-2)
    print >> sys.stderr, "  Check goPosParam (diff = ", (difx-goalx), "[m], ", (dify-goaly), "[m], ", (difth-goalth), "[deg])"
    print >> sys.stderr, "  => ", ret
    assert(ret)
    return ret

def calcVelListFromPosList(pos_list, dt):
    '''Calculate velocity list from position list.
    Element of pos_list and vel_list should be list like [0,0,0].
    '''
    vel_list=[]
    ppos=pos_list[0]
    for pos in pos_list:
        vel_list.append(map(lambda x, y: (x-y)/dt, pos, ppos));
        ppos=pos
    return vel_list

def checkTooLargeABCCogAcc (acc_thre = 5.0): # [m/s^2]
    '''Check ABC too large cog acceleration.
    This is used discontinuous cog trajectory.
    '''
    # Parse COG [m] and tm [s]
    cog_list=[]
    tm_list=[]
    for line in open("/tmp/test-abc-log.abst_refCogOut", "r"):
        tm_list.append(float(line.split(" ")[0]));
        cog_list.append(map(float, line.split(" ")[1:-1]));
    cog_list=cog_list[:-1000] # ?? Neglect latter elements
    dt = tm_list[1]-tm_list[0] # [s]
    # Calculate velocity and acceleration
    dcog_list=calcVelListFromPosList(cog_list, dt)
    ddcog_list=calcVelListFromPosList(dcog_list, dt)
    # Check max
    max_cogx_acc = max(map(lambda x : abs(x[0]), ddcog_list))
    max_cogy_acc = max(map(lambda x : abs(x[1]), ddcog_list))
    ret = (max_cogx_acc < acc_thre) and (max_cogy_acc < acc_thre)
    print >> sys.stderr, "  Check acc x = ", max_cogx_acc, ", y = ", max_cogy_acc, ", thre = ", acc_thre, "[m/s^2], ret = ", ret
    assert(ret)

def demoAutoBalancerFixFeet ():
    print >> sys.stderr, "1. AutoBalancer mode by fixing feet"
    hcf.abst_svc.startAutoBalancer(["rleg", "lleg"]);
    hcf.seq_svc.setJointAngles(arm_front_pose, 1.0)
    hcf.waitInterpolation()
    hcf.seq_svc.setJointAngles(initial_pose, 1.0)
    hcf.waitInterpolation()
    hcf.abst_svc.stopAutoBalancer();
    checkActualBaseAttitude()
    print >> sys.stderr, "  Start and Stop AutoBalancer by fixing feet=>OK"

def demoAutoBalancerFixFeetHands ():
    print >> sys.stderr, "2. AutoBalancer mode by fixing hands and feet"
    hcf.abst_svc.startAutoBalancer(autobalancer_limbs)
    hcf.seq_svc.setJointAngles(arm_front_pose, 1.0)
    hcf.waitInterpolation()
    hcf.seq_svc.setJointAngles(initial_pose, 1.0)
    hcf.waitInterpolation()
    hcf.abst_svc.stopAutoBalancer();
    checkActualBaseAttitude()
    print >> sys.stderr, "  Start and Stop AutoBalancer by fixing hands and feet=>OK"

def demoAutoBalancerGetParam():
    print >> sys.stderr, "3. getAutoBalancerParam"
    ret = hcf.abst_svc.getAutoBalancerParam()
    if ret[0]:
        print >> sys.stderr, "  getAutoBalancerParam() => OK"

def demoAutoBalancerSetParam():
    print >> sys.stderr, "4. setAutoBalancerParam"
    abcp=hcf.abst_svc.getAutoBalancerParam()[1]
    abcp.default_zmp_offsets = [[0.1,0,0], [0.1,0,0], [0,0,0], [0,0,0]]
    hcf.abst_svc.setAutoBalancerParam(abcp)
    print >> sys.stderr, "  default_zmp_offsets setting check in start and stop"
    hcf.abst_svc.startAutoBalancer(["rleg", "lleg"]);
    hcf.abst_svc.stopAutoBalancer();
    ret=hcf.abst_svc.getAutoBalancerParam()
    flag = (ret[0] and numpy.allclose(ret[1].default_zmp_offsets, abcp.default_zmp_offsets, 1e-6))
    if flag:
        print >> sys.stderr, "  setAutoBalancerParam() => OK"
    assert (flag), (ret[0], ret[1].default_zmp_offsets, abcp.default_zmp_offsets)
    abcp.default_zmp_offsets = [[0,0,0], [0,0,0], [0,0,0], [0,0,0]]
    hcf.abst_svc.setAutoBalancerParam(abcp)

def demoAutoBalancerTestPoses():
    print >> sys.stderr, "5. change base height, base rot x, base rot y, and upper body while AutoBalancer mode"
    hcf.abst_svc.startAutoBalancer(["rleg", "lleg"]);
    testPoseList(pose_list, initial_pose)
    hcf.abst_svc.stopAutoBalancer();
    checkActualBaseAttitude()

def demoAutoBalancerStartStopCheck():
    print >> sys.stderr, "6. start stop check"
    abcp=hcf.abst_svc.getAutoBalancerParam()[1]
    abcp.default_zmp_offsets = [[-0.05,0.05,0], [-0.05,0.05,0], [0,0,0], [0,0,0]]
    hcf.abst_svc.setAutoBalancerParam(abcp)
    hcf.setMaxLogLength(1500)
    for pose in pose_list:
        hcf.seq_svc.setJointAngles(pose, 1.0)
        hcf.waitInterpolation()
        hcf.clearLog()
        hcf.abst_svc.startAutoBalancer(["rleg", "lleg"]);
        hcf.abst_svc.stopAutoBalancer();
        hcf.saveLog("/tmp/test-samplerobot-abc-startstop-{0}".format(pose_list.index(pose)))
    abcp.default_zmp_offsets = [[0,0,0], [0,0,0], [0,0,0], [0,0,0]]
    hcf.abst_svc.setAutoBalancerParam(abcp)
    hcf.seq_svc.setJointAngles(initial_pose, 1.0)
    hcf.waitInterpolation()
    checkActualBaseAttitude()

def demoAutoBalancerBalanceAgainstHandForce():
    print >> sys.stderr, "7. balance against hand force"
    hcf.abst_svc.startAutoBalancer(["rleg", "lleg"]);
    hcf.seq_svc.setWrenches([0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,-50,0,0,0,], 1.0); # rhsensor
    hcf.waitInterpolation();
    hcf.seq_svc.setWrenches([0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0,0,0,], 1.0);
    hcf.waitInterpolation();
    hcf.abst_svc.stopAutoBalancer();
    checkActualBaseAttitude()

def demoAutoBalancerBalanceWithArms():
    print >> sys.stderr, "8. balance with arms"
    hcf.seq_svc.setJointAngles(four_legs_mode_pose, 1.0)
    hcf.waitInterpolation()
    abcp=hcf.abst_svc.getAutoBalancerParam()[1]
    abcp.leg_names = ['rleg', 'lleg', 'rarm', 'larm']
    hcf.abst_svc.setAutoBalancerParam(abcp)
    hcf.abst_svc.startAutoBalancer(autobalancer_limbs);
    print >> sys.stderr, "  startAutoBalancer with arms"
    hcf.abst_svc.stopAutoBalancer();
    print >> sys.stderr, "  stopAutoBalancer"
    abcp.leg_names = ['rleg', 'lleg']
    hcf.abst_svc.setAutoBalancerParam(abcp)
    checkActualBaseAttitude()
    hcf.seq_svc.setJointAngles(initial_pose, 1.0)
    hcf.waitInterpolation()

def demoGaitGeneratorBaseTformCheck ():
    print >> sys.stderr, "0. baseTform check"
    # Set parameter
    orig_ggp = hcf.abst_svc.getGaitGeneratorParam()[1]
    orig_abcp = hcf.abst_svc.getAutoBalancerParam()[1]
    hcf.co_svc.disableCollisionDetection()
    ggp = hcf.abst_svc.getGaitGeneratorParam()[1]
    ggp.stride_parameter = [0.2, 0.1, 20, 0.15]
    ggp.default_step_time = 0.5
    hcf.abst_svc.setGaitGeneratorParam(ggp)
    abcp = hcf.abst_svc.getAutoBalancerParam()[1]
    abcp.transition_time = 0.1
    hcf.abst_svc.setAutoBalancerParam(abcp)
    btf0 = checkParameterFromLog("baseTformOut", rtc_name="abst")
    # Check start ABC
    hcf.seq_svc.setJointAnglesSequenceFull([[0.000242, -0.403476, -0.000185, 0.832071, -0.427767, -6.928952e-05, 0.31129, -0.159481, -0.115399, -0.636277, 0.0, 0.0, 0.0, 0.000242, -0.403469, -0.000185, 0.832073, -0.427775, -6.928781e-05, 0.31129, 0.159481, 0.115399, -0.636277, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], # jvss
                                           [[0]*29], # vels
                                           [[0]*29], # torques
                                           [[btf0[0]+0.2+-0.014759, btf0[1]+-0.1+-4.336272e-05, 0.668138]], # poss
                                           [[-0.000245, -0.000862, 0.000171]], # rpys
                                           [[0]*3], # accs
                                           [[0.014052, 0.000203, -0.66798]], # zmps
                                           [[0]*6*4], # wrenchs
                                           [[1,1,0,0,1,1,1,1]], # optionals
                                           [0.5]); # tms
    hcf.waitInterpolation()
    btf1 = checkParameterFromLog("baseTformOut", rtc_name="abst")
    hcf.abst_svc.startAutoBalancer(autobalancer_limbs)
    btf2 = checkParameterFromLog("baseTformOut", rtc_name="abst")
    # Check stop ABC
    hcf.abst_svc.goPos(-0.2, 0.1, 0)
    hcf.abst_svc.waitFootSteps()
    btf3 = checkParameterFromLog("baseTformOut", rtc_name="abst")
    hcf.seq_svc.setJointAnglesSequenceFull([[0.000242, -0.403476, -0.000185, 0.832071, -0.427767, -6.928952e-05, 0.31129, -0.159481, -0.115399, -0.636277, 0.0, 0.0, 0.0, 0.000242, -0.403469, -0.000185, 0.832073, -0.427775, -6.928781e-05, 0.31129, 0.159481, 0.115399, -0.636277, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], # jvss
                                           [[0]*29], # vels
                                           [[0]*29], # torques
                                           [[btf0[0]+-0.014759, btf0[1]+-4.336272e-05, 0.668138]], # poss
                                           [[-0.000245, -0.000862, 0.000171]], # rpys
                                           [[0]*3], # accs
                                           [[0.014052, 0.000203, -0.66798]], # zmps
                                           [[0]*6*4], # wrenchs
                                           [[1,1,0,0,1,1,1,1]], # optionals
                                           [0.1]); # tms
    hcf.waitInterpolation()
    hcf.abst_svc.stopAutoBalancer()
    btf4 = checkParameterFromLog("baseTformOut", rtc_name="abst")
    # Finalize
    hcf.abst_svc.setGaitGeneratorParam(orig_ggp)
    hcf.abst_svc.setAutoBalancerParam(orig_abcp)
    hcf.co_svc.enableCollisionDetection()
    # Check values (currently pos x,y only 1[mm])
    startABC_OK = all(map (lambda x,y : abs(x-y)<1*1e-3, btf1[0:3], btf2[0:3]))
    stopABC_OK  = all(map (lambda x,y : abs(x-y)<1*1e-3, btf3[0:3], btf4[0:3]))
    print >> sys.stderr, "  before startABC = ", btf1[0:3], ", after startABC = ", btf2[0:3], ", diff = ", startABC_OK
    print >> sys.stderr, "  before stopABC  = ", btf3[0:3], ", after stopABC  = ", btf4[0:3], ", diff = ", stopABC_OK
    assert(startABC_OK and stopABC_OK)

def demoGaitGeneratorGoPos():
    print >> sys.stderr, "1. goPos"
    hcf.abst_svc.startAutoBalancer(autobalancer_limbs);
    # initialize dst_foot_midcoords
    hcf.abst_svc.goPos(0,0,0)
    hcf.abst_svc.waitFootSteps()
    # gopos check 1
    goalx=0.1;goaly=0.1;goalth=20.0
    prev_dst_foot_midcoords=hcf.abst_svc.getFootstepParam()[1].dst_foot_midcoords
    hcf.abst_svc.goPos(goalx, goaly, goalth)
    hcf.abst_svc.waitFootSteps()
    checkGoPosParam(goalx, goaly, goalth, prev_dst_foot_midcoords)
    # gopos check 2
    goalx=-0.1;goaly=-0.1;goalth=-10.0
    prev_dst_foot_midcoords=hcf.abst_svc.getFootstepParam()[1].dst_foot_midcoords
    hcf.abst_svc.goPos(goalx, goaly, goalth)
    hcf.abst_svc.waitFootSteps()
    checkGoPosParam(goalx, goaly, goalth, prev_dst_foot_midcoords)
    checkActualBaseAttitude()
    print >> sys.stderr, "  goPos()=>OK"

def demoGaitGeneratorGoVelocity():
    print >> sys.stderr, "2. goVelocity and goStop"
    print >> sys.stderr, "  goVelocity few steps"
    hcf.abst_svc.goVelocity(-0.1, -0.05, -20)
    time.sleep(1)
    hcf.abst_svc.goStop()
    checkActualBaseAttitude()
    print >> sys.stderr, "  goVelocity few steps=>OK"
    print >> sys.stderr, "  Check discontinuity of COG by checking too large COG acc."
    hcf.setMaxLogLength(10000)
    hcf.clearLog()
    hcf.abst_svc.goVelocity(0,0,0) # One step overwrite
    hcf.abst_svc.goStop()
    hcf.saveLog("/tmp/test-abc-log");
    checkTooLargeABCCogAcc()

def demoGaitGeneratorSetFootSteps():
    print >> sys.stderr, "3. setFootSteps"
    hcf.abst_svc.setFootSteps([OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0,-0.09,0], [1,0,0,0], "rleg")]),
                      OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0,0.09,0], [1,0,0,0], "lleg")])], 0)
    hcf.abst_svc.waitFootSteps()
    hcf.abst_svc.setFootSteps([OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0,-0.09,0], [1,0,0,0], "rleg")]),
                      OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.15,0.09,0], [1,0,0,0], "lleg")]),
                      OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.3,-0.09,0], [1,0,0,0], "rleg")]),
                      OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.3,0.09,0], [1,0,0,0], "lleg")])], 0)
    hcf.abst_svc.waitFootSteps()
    checkActualBaseAttitude()
    print >> sys.stderr, "  setFootSteps()=>OK"

def demoGaitGeneratorChangePoseWhileWalking():
    print >> sys.stderr, "4. Change base height, base rot x, base rot y, and upper body while walking"
    hcf.abst_svc.waitFootSteps()
    hcf.abst_svc.goVelocity(0,0,0)
    testPoseList(pose_list, initial_pose)
    hcf.abst_svc.goStop()
    checkActualBaseAttitude()

def demoGaitGeneratorGetParam():
    print >> sys.stderr, "5. getGaitGeneratorParam"
    ret = hcf.abst_svc.getGaitGeneratorParam()
    if ret[0]:
        print >> sys.stderr, "  getGaitGeneratorParam() => OK"

def demoGaitGeneratorSetParam():
    print >> sys.stderr, "6. setGaitGeneratorParam"
    ggp_org = hcf.abst_svc.getGaitGeneratorParam()[1]
    ggp = hcf.abst_svc.getGaitGeneratorParam()[1]
    ggp.default_step_time = 0.9
    ggp.default_step_height = 0.15
    ggp.default_double_support_ratio = 0.4
    ggp.swing_trajectory_delay_time_offset = 0.20
    ggp.default_orbit_type = OpenHRP.AutoBalanceStabilizerService.RECTANGLE;
    hcf.abst_svc.setGaitGeneratorParam(ggp)
    ret = hcf.abst_svc.getGaitGeneratorParam()
    if ret[0] and ret[1].default_step_time == ggp.default_step_time and ret[1].default_step_height == ggp.default_step_height and ret[1].default_double_support_ratio == ggp.default_double_support_ratio and ret[1].default_orbit_type == ggp.default_orbit_type:
        print >> sys.stderr, "  setGaitGeneratorParam() => OK"
    hcf.abst_svc.goPos(0.2,0,0)
    hcf.abst_svc.waitFootSteps()
    hcf.abst_svc.setGaitGeneratorParam(ggp_org) # revert parameter

def demoGaitGeneratorNonDefaultStrideStop():
    print >> sys.stderr, "7. non-default stride"
    hcf.abst_svc.setFootSteps([OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0,-0.09,0], [1,0,0,0], "rleg")]),
                      OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.15,0.09,0], [1,0,0,0], "lleg")])], 0)
    hcf.abst_svc.waitFootSteps()
    hcf.abst_svc.setFootSteps([OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0,-0.09,0], [1,0,0,0], "rleg")]),
                      OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0,0.09,0], [1,0,0,0], "lleg")])], 0)
    hcf.abst_svc.waitFootSteps()
    checkActualBaseAttitude()
    print >> sys.stderr, "  Non default Stride()=>OK"

def demoGaitGeneratorToeHeelContact():
    print >> sys.stderr, "8. Use toe heel contact"
    ggp=hcf.abst_svc.getGaitGeneratorParam()[1];
    ggp.toe_pos_offset_x = 1e-3*182.0;
    ggp.heel_pos_offset_x = 1e-3*-72.0;
    ggp.toe_zmp_offset_x = 1e-3*182.0;
    ggp.heel_zmp_offset_x = 1e-3*-72.0;
    ggp.toe_angle = 20;
    ggp.heel_angle = 10;
    hcf.abst_svc.setGaitGeneratorParam(ggp);
    hcf.abst_svc.goPos(0.3, 0, 0);
    hcf.abst_svc.waitFootSteps()
    ggp.toe_angle = 0;
    ggp.heel_angle = 0;
    hcf.abst_svc.setGaitGeneratorParam(ggp);
    checkActualBaseAttitude()
    print >> sys.stderr, "  Toe heel contact=>OK"

def demoGaitGeneratorStopStartSyncCheck():
    print >> sys.stderr, "9. Stop and start auto balancer sync check2"
    print >> sys.stderr, "  Check 9-1 Sync after setFootSteps"
    hcf.abst_svc.startAutoBalancer(autobalancer_limbs);
    hcf.abst_svc.setFootSteps([OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0,-0.09,0], [1,0,0,0], "rleg")]),
                      OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.1,0.09,0], [1,0,0,0], "lleg")])], 0);
    hcf.abst_svc.waitFootSteps();
    hcf.abst_svc.stopAutoBalancer();
    print >> sys.stderr, "    Sync after setFootSteps => OK"
    print >> sys.stderr, "  Check 9-2 Sync from setJointAngles at the beginning"
    open_stride_pose = [0.00026722677758058496, -0.3170503560247552, -0.0002054613599000865, 0.8240549352035262, -0.5061434785447525, -8.67443660992421e-05, 0.3112899999999996, -0.15948099999999998, -0.11539900000000003, -0.6362769999999993, 0.0, 0.0, 0.0, 0.00023087433689200683, -0.4751295978345554, -0.00021953834197007937, 0.8048588066686679, -0.3288687069275527, -8.676469399681631e-05, 0.3112899999999996, 0.15948099999999998, 0.11539900000000003, -0.6362769999999993, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    hcf.seq_svc.setJointAngles(open_stride_pose, 2.0);
    hcf.waitInterpolation();
    hcf.abst_svc.startAutoBalancer(autobalancer_limbs);
    hcf.abst_svc.setFootSteps([OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0,-0.09,0], [1,0,0,0], "rleg")]),
                      OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.1,0.09,0], [1,0,0,0], "lleg")])], 0);
    hcf.abst_svc.waitFootSteps();
    hcf.abst_svc.stopAutoBalancer();
    print >> sys.stderr, "    Sync from setJointAngle at the beginning => OK"
    print >> sys.stderr, "  Check 9-3 Sync from setJointAngles"
    hcf.abst_svc.startAutoBalancer(autobalancer_limbs);
    hcf.seq_svc.setJointAngles(initial_pose, 2.0);
    hcf.waitInterpolation();
    hcf.abst_svc.stopAutoBalancer();
    print >> sys.stderr, "    Sync from setJointAngle => OK"

def demoGaitGeneratorEmergencyStop():
    print >> sys.stderr, "10. Emergency stop"
    hcf.abst_svc.startAutoBalancer(autobalancer_limbs);
    hcf.abst_svc.goPos(0,0,90);
    print >> sys.stderr, "  Start goPos and wait for 4 steps"
    for idx in range(4): # Wait for 4 steps including initial double support phase
        # Wait for 1 steps
        hcf.seq_svc.setJointAngles(initial_pose, hcf.abst_svc.getGaitGeneratorParam()[1].default_step_time);
        hcf.waitInterpolation();
    print >> sys.stderr, "  Emergency stoping"
    hcf.abst_svc.emergencyStop();
    print >> sys.stderr, "  Align foot steps"
    hcf.abst_svc.goPos(0,0,0);
    checkActualBaseAttitude()

def demoGaitGeneratorGetRemainingSteps():
    print >> sys.stderr, "11. Get remaining foot steps"
    hcf.abst_svc.goPos(0.3,0.1,15);
    fslist=hcf.abst_svc.getRemainingFootstepSequence()[1]
    while fslist != []:
        fslist=hcf.abst_svc.getRemainingFootstepSequence()[1]
        print >> sys.stderr, "  Remaining footstep ", len(fslist)
        # Wait for 1 step
        hcf.seq_svc.setJointAngles(initial_pose, hcf.abst_svc.getGaitGeneratorParam()[1].default_step_time);
        hcf.waitInterpolation();
    checkActualBaseAttitude()

def demoGaitGeneratorChangeStepParam():
    print >> sys.stderr, "12. Change step param with setFootSteps"
    ggp_org=hcf.abst_svc.getGaitGeneratorParam()[1];
    # dummy setting
    ggp=hcf.abst_svc.getGaitGeneratorParam()[1];
    ggp.toe_angle = 50;
    ggp.heel_angle = 50;
    hcf.abst_svc.setGaitGeneratorParam(ggp);
    hcf.abst_svc.setFootStepsWithParam([OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0,-0.09,0], [1,0,0,0], "rleg")]),
                               OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.1,0.09,0], [1,0,0,0], "lleg")]),
                               OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.2,-0.09,0], [1,0,0,0], "rleg")]),
                               OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.2,0.09,0], [1,0,0,0], "lleg")])],
                              [OpenHRP.AutoBalanceStabilizerService.StepParams([OpenHRP.AutoBalanceStabilizerService.StepParam(step_height=0.0, step_time=1.0, toe_angle=0.0, heel_angle=0.0)]),
                               OpenHRP.AutoBalanceStabilizerService.StepParams([OpenHRP.AutoBalanceStabilizerService.StepParam(step_height=0.05, step_time=2.0, toe_angle=0.0, heel_angle=0.0)]),
                               OpenHRP.AutoBalanceStabilizerService.StepParams([OpenHRP.AutoBalanceStabilizerService.StepParam(step_height=0.05, step_time=1.0, toe_angle=0.0, heel_angle=0.0)]),
                               OpenHRP.AutoBalanceStabilizerService.StepParams([OpenHRP.AutoBalanceStabilizerService.StepParam(step_height=0.05, step_time=2.0, toe_angle=0.0, heel_angle=0.0)])], 0)
    hcf.abst_svc.waitFootSteps()
    hcf.abst_svc.setFootStepsWithParam([OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0,-0.09,0], [1,0,0,0], "rleg")]),
                               OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.1,0.09,0], [1,0,0,0], "lleg")]),
                               OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.2,-0.09,0], [1,0,0,0], "rleg")]),
                               OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.2,0.09,0], [1,0,0,0], "lleg")])],
                              [OpenHRP.AutoBalanceStabilizerService.StepParams([OpenHRP.AutoBalanceStabilizerService.StepParam(step_height=0.0, step_time=1.0, toe_angle=0.0, heel_angle=0.0)]),
                               OpenHRP.AutoBalanceStabilizerService.StepParams([OpenHRP.AutoBalanceStabilizerService.StepParam(step_height=0.1, step_time=1.0, toe_angle=0.0, heel_angle=0.0)]),
                               OpenHRP.AutoBalanceStabilizerService.StepParams([OpenHRP.AutoBalanceStabilizerService.StepParam(step_height=0.05, step_time=1.0, toe_angle=0.0, heel_angle=0.0)]),
                               OpenHRP.AutoBalanceStabilizerService.StepParams([OpenHRP.AutoBalanceStabilizerService.StepParam(step_height=0.1, step_time=1.0, toe_angle=0.0, heel_angle=0.0)])], 0)
    hcf.abst_svc.waitFootSteps()
    hcf.abst_svc.setFootStepsWithParam([OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0,-0.09,0], [1,0,0,0], "rleg")]),
                               OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.1,0.09,0], [1,0,0,0], "lleg")]),
                               OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.2,-0.09,0], [1,0,0,0], "rleg")]),
                               OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.2,0.09,0], [1,0,0,0], "lleg")])],
                              [OpenHRP.AutoBalanceStabilizerService.StepParams([OpenHRP.AutoBalanceStabilizerService.StepParam(step_height=0.0, step_time=1.0, toe_angle=0.0, heel_angle=0.0)]),
                               OpenHRP.AutoBalanceStabilizerService.StepParams([OpenHRP.AutoBalanceStabilizerService.StepParam(step_height=0.05, step_time=1.0, toe_angle=0.0, heel_angle=0.0)]),
                               OpenHRP.AutoBalanceStabilizerService.StepParams([OpenHRP.AutoBalanceStabilizerService.StepParam(step_height=0.05, step_time=1.0, toe_angle=20.0, heel_angle=5.0)]),
                               OpenHRP.AutoBalanceStabilizerService.StepParams([OpenHRP.AutoBalanceStabilizerService.StepParam(step_height=0.05, step_time=1.0, toe_angle=10.0, heel_angle=10.0)])], 0)
    hcf.abst_svc.waitFootSteps()
    hcf.abst_svc.setGaitGeneratorParam(ggp_org);
    checkActualBaseAttitude()

def demoGaitGeneratorOverwriteFootsteps(overwrite_offset_idx = 1):
    print >> sys.stderr, "13. Overwrite footsteps during walking."
    hcf.abst_svc.startAutoBalancer(autobalancer_limbs)
    demoGaitGeneratorOverwriteFootstepsBase("x", overwrite_offset_idx, True) # Overwrite by X direction foot steps
    hcf.seq_svc.setJointAngles(initial_pose, 1.0*overwrite_offset_idx)
    hcf.seq_svc.waitInterpolation()
    demoGaitGeneratorOverwriteFootstepsBase("y", overwrite_offset_idx, True) # Overwrite by Y direction foot steps
    hcf.seq_svc.setJointAngles(initial_pose, 1.0*overwrite_offset_idx)
    hcf.seq_svc.waitInterpolation()
    demoGaitGeneratorOverwriteFootstepsBase("x", overwrite_offset_idx, True) # Overwrite by X direction foot steps
    hcf.abst_svc.waitFootSteps()
    checkActualBaseAttitude()

def demoGaitGeneratorOverwriteFootstepsBase(axis, overwrite_offset_idx = 1, init_fs = False):
    if init_fs:
        hcf.abst_svc.setFootSteps([OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0,  -0.09,0], [1,0,0,0], "rleg")]),
                          OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.1, 0.09,0], [1,0,0,0], "lleg")]),
                          OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.2,-0.09,0], [1,0,0,0], "rleg")]),
                          OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.3, 0.09,0], [1,0,0,0], "lleg")]),
                          OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.4,-0.09,0], [1,0,0,0], "rleg")]),
                          OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.4, 0.09,0], [1,0,0,0], "lleg")])], 0);
    print >> sys.stderr, "  Overwrite footsteps ", overwrite_offset_idx
    # Get remaining footstep
    [remain_fs, current_fs_idx]=hcf.abst_svc.getRemainingFootstepSequence()[1:]
    #print >> sys.stderr, remain_fs
    print >> sys.stderr, "    Remaining legs = ", map(lambda fs : fs.leg, remain_fs)
    print >> sys.stderr, "    Remaining idx  = ", map(lambda idx : current_fs_idx+idx, range(len(remain_fs)))
    # Footstep index to be overwritten
    overwrite_fs_idx = current_fs_idx + overwrite_offset_idx
    print >> sys.stderr, "    Overwrite index = ",overwrite_fs_idx, ", leg = ", remain_fs[overwrite_offset_idx].leg
    # Calc new footsteps
    import numpy
    support_fs = remain_fs[overwrite_offset_idx-1] # support fs before overwritten fs
    if axis == "x":
        pos_offset = [0.1, 0, 0]
        pos_offset2 = [0.2, 0, 0]
    else:
        pos_offset = [0, (0.1 if support_fs.leg =='rleg' else -0.1), 0]
        pos_offset2 = pos_offset
    fpos1=list(numpy.array(support_fs.pos) + numpy.array([0, 2.0*(0.09 if support_fs.leg =='rleg' else -0.09) ,0]) + numpy.array(pos_offset))
    fpos2=list(numpy.array(support_fs.pos) + numpy.array(pos_offset))
    fpos3=list(numpy.array(support_fs.pos) + numpy.array([0, 2.0*(0.09 if support_fs.leg =='rleg' else -0.09) ,0]) + numpy.array(pos_offset2))
    fpos4=list(numpy.array(support_fs.pos) + numpy.array(pos_offset2))
    new_fs =[OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep(support_fs.pos, [1,0,0,0], support_fs.leg)]),
             OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep(fpos1,          [1,0,0,0], "lleg" if support_fs.leg =='rleg' else "rleg")]),
             OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep(fpos2,          [1,0,0,0], support_fs.leg)]),
             OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep(fpos3,          [1,0,0,0], "lleg" if support_fs.leg =='rleg' else "rleg")]),
             OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep(fpos4,          [1,0,0,0], support_fs.leg)])]
    hcf.abst_svc.setFootSteps(new_fs, overwrite_fs_idx);

def demoGaitGeneratorFixHand():
    print >> sys.stderr, "14. Fix arm walking"
    hcf.abst_svc.stopAutoBalancer()
    hcf.abst_svc.startAutoBalancer(autobalancer_limbs)
    # Set pose
    abcp=hcf.abst_svc.getAutoBalancerParam()[1]
    abcp.default_zmp_offsets=[[0.01, 0.0, 0.0], [0.01, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]] # Setting default_zmp_offsets is not necessary for fix mode. Just for debugging for default_zmp_offsets in hand fix mode.
    hcf.abst_svc.setAutoBalancerParam(abcp)
    dualarm_push_pose=[-3.998549e-05,-0.710564,-0.000264,1.41027,-0.680767,-2.335251e-05,-0.541944,-0.091558,0.122667,-1.02616,-1.71287,-0.056837,1.5708,-3.996804e-05,-0.710511,-0.000264,1.41016,-0.680706,-2.333505e-05,-0.542,0.091393,-0.122578,-1.02608,1.71267,-0.05677,-1.5708,0.006809,0.000101,-0.000163]
    hcf.seq_svc.setJointAngles(dualarm_push_pose, 1.0)
    hcf.waitInterpolation()
    print >> sys.stderr, "  Walk without fixing arm"
    abcp=hcf.abst_svc.getAutoBalancerParam()[1]
    abcp.is_hand_fix_mode=False
    hcf.abst_svc.setAutoBalancerParam(abcp)
    hcf.abst_svc.goPos(0.3,0,0)
    hcf.abst_svc.waitFootSteps()
    hcf.abst_svc.goPos(0,0.2,0)
    hcf.abst_svc.waitFootSteps()
    hcf.abst_svc.goPos(0,0,30)
    hcf.abst_svc.waitFootSteps()
    print >> sys.stderr, "  Walk with fixing arm"
    abcp=hcf.abst_svc.getAutoBalancerParam()[1]
    abcp.is_hand_fix_mode=True
    hcf.abst_svc.setAutoBalancerParam(abcp)
    hcf.abst_svc.goPos(0.3,0,0)
    hcf.abst_svc.waitFootSteps()
    hcf.abst_svc.goPos(0,-0.2,0)
    hcf.abst_svc.waitFootSteps()
    hcf.abst_svc.goPos(0,0,-30)
    hcf.abst_svc.waitFootSteps()
    abcp=hcf.abst_svc.getAutoBalancerParam()[1]
    abcp.is_hand_fix_mode=False
    abcp.default_zmp_offsets=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    hcf.abst_svc.setAutoBalancerParam(abcp)
    ref_rpy = checkParameterFromLog("baseRpyOut", rtc_name="abst")
    hcf.abst_svc.stopAutoBalancer()
    checkActualBaseAttitude(ref_rpy)
    print >> sys.stderr, "  Fix hand=>OK"

def demoGaitGeneratorOverwriteCurrentFootstep():
    print >> sys.stderr, "15. Overwrite current footstep"
    hcf.seq_svc.setJointAngles(initial_pose, 1.0)
    hcf.waitInterpolation()
    hcf.abst_svc.startAutoBalancer(autobalancer_limbs)
    # decrease zmp weight for arms
    orig_ggp = hcf.abst_svc.getGaitGeneratorParam()[1]
    ggp = hcf.abst_svc.getGaitGeneratorParam()[1]
    ggp.overwritable_footstep_index_offset = 0
    ggp.default_orbit_type=OpenHRP.AutoBalanceStabilizerService.RECTANGLE
    hcf.abst_svc.setGaitGeneratorParam(ggp)
    # start walking
    hcf.abst_svc.goVelocity(0,0,0);
    hcf.seq_svc.setJointAngles(initial_pose, 2.0);hcf.waitInterpolation() #  wait 2 step using dummy waitInterpolation
    hcf.abst_svc.goVelocity(0.1,0,0);
    hcf.seq_svc.setJointAngles(initial_pose, 2.0);hcf.waitInterpolation() #  wait 2 step using dummy waitInterpolation
    hcf.abst_svc.goVelocity(0,0.1,0);
    hcf.seq_svc.setJointAngles(initial_pose, 2.0);hcf.waitInterpolation() #  wait 2 step using dummy waitInterpolation
    hcf.abst_svc.goStop()
    checkActualBaseAttitude()
    print >> sys.stderr, "  Overwrite current footstep=>OK"
    # reset params
    hcf.abst_svc.setGaitGeneratorParam(orig_ggp)

def demoGaitGeneratorGoPosOverwrite():
    print >> sys.stderr, "16. goPos overwriting"
    hcf.abst_svc.startAutoBalancer(autobalancer_limbs);
    print >> sys.stderr, "  Overwrite goPos by goPos"
    # Initialize dst_foot_midcoords
    hcf.abst_svc.goPos(0,0.001,0);
    hcf.abst_svc.waitFootSteps();
    goalx=0.3;goaly=0.1;goalth=15.0
    prev_dst_foot_midcoords=hcf.abst_svc.getFootstepParam()[1].dst_foot_midcoords
    hcf.abst_svc.goPos(0.2,-0.1,-5) # initial gopos
    hcf.seq_svc.setJointAngles(initial_pose, 2.0);hcf.waitInterpolation() #  wait 2 step using dummy waitInterpolation
    hcf.abst_svc.goPos(goalx,goaly,goalth) # overwrite gopos
    hcf.abst_svc.waitFootSteps()
    checkGoPosParam(goalx, goaly, goalth, prev_dst_foot_midcoords)
    print >> sys.stderr, "  Overwrite setFootSteps by goPos"
    prev_dst_foot_midcoords=hcf.abst_svc.getFootstepParam()[1].dst_foot_midcoords
    hcf.abst_svc.setFootSteps([OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0,-0.09,0], [1,0,0,0], "rleg")]),
                      OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.1,0.09,0], [1,0,0,0], "lleg")]),
                      OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.2,-0.09,0], [1,0,0,0], "rleg")]),
                      OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.3,0.09,0], [1,0,0,0], "lleg")]),
                      OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.3,-0.09,0], [1,0,0,0], "rleg")])
                      ], 0) # initial setfootsteps
    hcf.seq_svc.setJointAngles(initial_pose, 2.0);hcf.waitInterpolation() #  wait 2 step using dummy waitInterpolation
    hcf.abst_svc.goPos(goalx,goaly,goalth) # overwrite gopos
    hcf.abst_svc.waitFootSteps()
    checkGoPosParam(goalx, goaly, goalth, prev_dst_foot_midcoords)

def demoGaitGeneratorGrasplessManipMode():
    print >> sys.stderr, "17. Graspless manip mode"
    hcf.abst_svc.startAutoBalancer(autobalancer_limbs);
    # Initialize and pose define
    dualarm_push_pose=[-3.998549e-05,-0.710564,-0.000264,1.41027,-0.680767,-2.335251e-05,-0.541944,-0.091558,0.122667,-1.02616,-1.71287,-0.056837,1.5708,-3.996804e-05,-0.710511,-0.000264,1.41016,-0.680706,-2.333505e-05,-0.542,0.091393,-0.122578,-1.02608,1.71267,-0.05677,-1.5708,0.006809,0.000101,-0.000163]
    hcf.seq_svc.setJointAngles(dualarm_push_pose, 0.5)
    hcf.waitInterpolation()
    # hands 50[mm] fwd from dualarm_push_pose
    av_fwd = [-5.579249e-05,-0.760285,-0.000277,1.44619,-0.660772,-2.615057e-05,-0.7752,-0.080815,0.116555,-0.935667,-1.70514,-0.045373,1.309,-5.577374e-05,-0.760232,-0.000277,1.44608,-0.660715,-2.613350e-05,-0.77525,0.080663,-0.116463,-0.935597,1.70494,-0.045325,-1.309,0.157668,0.000123,-0.000152]
    # hands 50[mm] bwd from dualarm_push_pose
    av_bwd = [-1.901820e-05,-0.641174,-0.00025,1.36927,-0.717047,-2.260319e-05,-0.305537,-0.099557,0.134675,-1.04208,-1.72497,-0.065256,1.309,-1.900236e-05,-0.641122,-0.00025,1.36915,-0.71698,-2.258509e-05,-0.305624,0.099383,-0.134605,-1.04197,1.72476,-0.06517,-1.309,-0.22394,5.625198e-05,-0.000165]
    # hands 50[mm] right from dualarm_push_pose
    av_right = [-0.005678,-0.711398,0.006148,1.40852,-0.678974,0.011103,-0.575284,-0.179786,0.092155,-0.999366,-1.74805,0.048205,1.309,-0.005686,-0.710309,0.006143,1.4098,-0.681345,0.011103,-0.520691,0.002033,-0.154878,-1.05585,1.6731,-0.161177,-1.309,0.015053,0.024788,-0.023196]
    # hands 50[mm] left from dualarm_push_pose
    av_left = [0.005607,-0.71036,-0.006671,1.40991,-0.681404,-0.011151,-0.52064,-0.002193,0.154967,-1.05593,-1.67329,-0.161246,1.309,0.005598,-0.711343,-0.006677,1.4084,-0.67891,-0.011151,-0.575342,0.179622,-0.092066,-0.999287,1.74785,0.04827,-1.309,0.015056,-0.024583,0.02287]
    # hands 10[deg] right turn from dualarm_push_pose
    av_rturn = [0.023512,-0.71245,0.014216,1.40899,-0.677868,-0.01575,-0.462682,0.040789,0.154221,-1.10667,-1.66067,-0.2349,1.309,0.023442,-0.708029,0.014161,1.40823,-0.68153,-0.015763,-0.61987,0.217174,-0.105089,-0.949927,1.75163,0.120793,-1.309,0.013747,-0.058774,-0.084435]
    # hands 10[deg] left turn from dualarm_push_pose
    av_lturn = [-0.023522,-0.708079,-0.014689,1.40835,-0.681597,0.015717,-0.619803,-0.217337,0.105179,-0.950004,-1.75184,0.120733,1.309,-0.02359,-0.712393,-0.014744,1.40889,-0.677813,0.0157,-0.462722,-0.040951,-0.154135,-1.10659,1.66048,-0.234826,-1.309,0.013715,0.058979,0.084109]
    # parameter setting
    org_abcp=hcf.abst_svc.getAutoBalancerParam()[1]
    abcp=hcf.abst_svc.getAutoBalancerParam()[1]
    abcp.graspless_manip_mode=True
    abcp.is_hand_fix_mode=True
    abcp.graspless_manip_reference_trans_rot=[1.0, 0.0, 0.0, 1.365307e-06] # trans rot for dualarm_push_pose
    abcp.graspless_manip_reference_trans_pos=[0.450037, 1.049436e-06, 0.869818] # trans pos for dualarm_push_pose
    abcp.graspless_manip_p_gain=[1,1,1]
    hcf.abst_svc.setAutoBalancerParam(abcp)
    hcf.co_svc.disableCollisionDetection()
    # Check one foot_midcoords movement
    gv_pose_list = [av_fwd, av_bwd, av_left, av_right, av_lturn, av_rturn]
    ref_footmid_diff = [[50*1e-3,0,0],
                        [-50*1e-3,0,0],
                        [0, 50*1e-3,0],
                        [0,-50*1e-3,0],
                        [0,0, 10],
                        [0,0,-10]]
    ret=True
    hcf.abst_svc.waitFootSteps()
    for idx in range(len(gv_pose_list)):
        pose = gv_pose_list[idx]
        prev_dst_foot_midcoords=hcf.abst_svc.getFootstepParam()[1].dst_foot_midcoords
        yvel = -0.0001 if (idx%2==0) else 0.0001 # Iff even number test, start with rleg. Otherwise, lleg.
        hcf.abst_svc.goVelocity(0,yvel,0);
        hcf.seq_svc.setJointAngles(pose, 0.4)
        hcf.waitInterpolation()
        hcf.seq_svc.setJointAngles(pose, 1.6);hcf.waitInterpolation() # Dummy 2step
        hcf.abst_svc.goStop()
        diff=numpy.array(ref_footmid_diff[idx])-numpy.array(calcDiffFootMidCoords(prev_dst_foot_midcoords))
        if idx == 4 or idx == 5:
            tmpret = abs(diff[2]) < 1.0 # TODO, check pos
        else:
            tmpret = abs(diff[0]) < 1e-3 and abs(diff[1]) < 1e-3 and abs(diff[2]) < 1.0
        ret = ret and tmpret
        print >> sys.stderr, "  ret = ", tmpret, ", diff = ", diff
    # Finishing
    if ret:
        print >> sys.stderr, "  total is OK"
    assert(ret)
    hcf.co_svc.enableCollisionDetection()
    hcf.seq_svc.setJointAngles(dualarm_push_pose, 0.5)
    hcf.waitInterpolation()
    hcf.abst_svc.setAutoBalancerParam(org_abcp)

def demoGaitGeneratorSetFootStepsWithArms():
    print >> sys.stderr, "18. Trot Walking"
    hcf.abst_svc.stopAutoBalancer()
    hcf.seq_svc.setJointAngles(four_legs_mode_pose, 1.0)
    hcf.waitInterpolation()
    # use arms
    orig_abcp = hcf.abst_svc.getAutoBalancerParam()[1]
    abcp = hcf.abst_svc.getAutoBalancerParam()[1]
    abcp.leg_names = ['rleg', 'lleg', 'rarm', 'larm']
    hcf.abst_svc.setAutoBalancerParam(abcp)
    # decrease zmp weight for arms
    orig_ggp = hcf.abst_svc.getGaitGeneratorParam()[1]
    ggp = hcf.abst_svc.getGaitGeneratorParam()[1]
    ggp.zmp_weight_map = [1.0, 1.0, 0.01, 0.01]
    ggp.default_step_height = 0.01
    hcf.abst_svc.setGaitGeneratorParam(ggp)
    # start walking
    hcf.abst_svc.startAutoBalancer(autobalancer_limbs)
    hcf.abst_svc.setFootSteps([OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0,-0.09,0], [1,0,0,0], "rleg"),
                                                             OpenHRP.AutoBalanceStabilizerService.Footstep([0.23,0.21,0.86], [1,0,0,0], "larm")]),
                      OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0,0.09,0], [1,0,0,0], "lleg"),
                                                             OpenHRP.AutoBalanceStabilizerService.Footstep([0.23,-0.21,0.86], [1,0,0,0], "rarm")])], 0)
    hcf.abst_svc.waitFootSteps()
    checkActualBaseAttitude()
    print >> sys.stderr, "  setFootSteps()=>OK"
    # reset params
    hcf.abst_svc.stopAutoBalancer()
    hcf.abst_svc.setAutoBalancerParam(orig_abcp)
    hcf.abst_svc.setGaitGeneratorParam(orig_ggp)
    hcf.abst_svc.startAutoBalancer(autobalancer_limbs)

def demoGaitGeneratorChangeStrideLimitationType():
    print >> sys.stderr, "19. Change stride limitation type to CIRCLE"
    hcf.abst_svc.startAutoBalancer(autobalancer_limbs);
    # initialize dst_foot_midcoords
    hcf.abst_svc.goPos(0,0,0)
    hcf.abst_svc.waitFootSteps()
    # set params
    orig_ggp = hcf.abst_svc.getGaitGeneratorParam()[1]
    ggp = hcf.abst_svc.getGaitGeneratorParam()[1]
    ggp.stride_limitation_type = OpenHRP.AutoBalanceStabilizerService.CIRCLE
    ggp.stride_limitation_for_circle_type = [0.15, 0.25, 10, 0.1, 0.1]
    ggp.leg_margin = [182.0*1e-3, 72.0*1e-3, 71.12*1e-3, 71.12*1e-3]
    hcf.abst_svc.setGaitGeneratorParam(ggp)
    # gopos check 1
    goalx=0.3;goaly=0.3;goalth=20.0
    prev_dst_foot_midcoords=hcf.abst_svc.getFootstepParam()[1].dst_foot_midcoords
    hcf.abst_svc.goPos(goalx, goaly, goalth)
    hcf.abst_svc.waitFootSteps()
    checkGoPosParam(goalx, goaly, goalth, prev_dst_foot_midcoords)
    # gopos check 2
    goalx=-0.3;goaly=-0.3;goalth=-10.0
    prev_dst_foot_midcoords=hcf.abst_svc.getFootstepParam()[1].dst_foot_midcoords
    hcf.abst_svc.goPos(goalx, goaly, goalth)
    hcf.abst_svc.waitFootSteps()
    checkGoPosParam(goalx, goaly, goalth, prev_dst_foot_midcoords)
    checkActualBaseAttitude()
    print >> sys.stderr, "  Change stride limitation type to CIRCLE=>OK"
    # reset params
    hcf.abst_svc.setGaitGeneratorParam(orig_ggp)

def demoStandingPosResetting():
    print >> sys.stderr, "demoStandingPosResetting"
    hcf.abst_svc.goPos(0,0,math.degrees(-1*checkParameterFromLog("WAIST")[5])); # Rot yaw
    hcf.abst_svc.waitFootSteps()
    hcf.abst_svc.goPos(0,-1*checkParameterFromLog("WAIST")[1],0); # Pos Y
    hcf.abst_svc.waitFootSteps()

def demoAutoBalancer():
    initAutoBalancer()
    # sample for AutoBalancer mode
    demoAutoBalancerFixFeet()
    demoAutoBalancerFixFeetHands()
    demoAutoBalancerGetParam()
    demoAutoBalancerSetParam()
    demoAutoBalancerTestPoses()
    demoAutoBalancerStartStopCheck()
    demoAutoBalancerBalanceAgainstHandForce()
    demoAutoBalancerBalanceWithArms()
    # sample for walk pattern generation by AutoBalancer RTC
    demoGaitGeneratorBaseTformCheck()
    demoGaitGeneratorGoPos()
    demoGaitGeneratorGoVelocity()
    demoGaitGeneratorSetFootSteps()
    demoGaitGeneratorChangePoseWhileWalking()
    demoGaitGeneratorGetParam()
    demoGaitGeneratorSetParam()
    demoGaitGeneratorNonDefaultStrideStop()
    demoGaitGeneratorToeHeelContact()
    demoGaitGeneratorStopStartSyncCheck()
    demoGaitGeneratorEmergencyStop()
    demoGaitGeneratorGetRemainingSteps()
    demoGaitGeneratorChangeStepParam()
    demoGaitGeneratorOverwriteFootsteps()
    demoGaitGeneratorOverwriteFootsteps(2)
    #demoStandingPosResetting()
    demoGaitGeneratorFixHand()
    demoGaitGeneratorOverwriteCurrentFootstep()
    demoGaitGeneratorGoPosOverwrite()
    demoGaitGeneratorGrasplessManipMode()
    demoGaitGeneratorSetFootStepsWithArms()
    demoGaitGeneratorChangeStrideLimitationType()

def initStabilizer():
    # on < 315.5.0 this outputs huge error log message
    hcf.seq_svc.setJointAngles(initial_pose, 2.0)
    hcf.seq_svc.waitInterpolation()
    # Remove offset
    for sen in ["rfsensor", "lfsensor"]:
        ofp=hcf.rmfo_svc.getForceMomentOffsetParam(sen)[1]
        ofp.link_offset_mass=1.9
        ofp.link_offset_centroid=[0.08, 0, -0.03]
        hcf.rmfo_svc.setForceMomentOffsetParam(sen, ofp)

def demoGetStabilizerParam():
    print >> sys.stderr, "1. getStabilizerParam"
    stp = hcf.abst_svc.getStabilizerParam()
    print >> sys.stderr, "  getStabilizerParam() => OK"

def demoSetStabilizerParam():
    print >> sys.stderr, "2. setStabilizerParam"
    stp_org = hcf.abst_svc.getStabilizerParam()
    # for tpcc
    stp_org.k_tpcc_p=[0.2, 0.2]
    stp_org.k_tpcc_x=[4.0, 4.0]
    stp_org.k_brot_p=[0.0, 0.0]
    # for eefm
    tmp_leg_inside_margin=71.12*1e-3
    tmp_leg_outside_margin=71.12*1e-3
    tmp_leg_front_margin=182.0*1e-3
    tmp_leg_rear_margin=72.0*1e-3
    rleg_vertices = [OpenHRP.AutoBalanceStabilizerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, tmp_leg_inside_margin]),
                     OpenHRP.AutoBalanceStabilizerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, -1*tmp_leg_outside_margin]),
                     OpenHRP.AutoBalanceStabilizerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, -1*tmp_leg_outside_margin]),
                     OpenHRP.AutoBalanceStabilizerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, tmp_leg_inside_margin])]
    lleg_vertices = [OpenHRP.AutoBalanceStabilizerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, tmp_leg_outside_margin]),
                     OpenHRP.AutoBalanceStabilizerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, -1*tmp_leg_inside_margin]),
                     OpenHRP.AutoBalanceStabilizerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, -1*tmp_leg_inside_margin]),
                     OpenHRP.AutoBalanceStabilizerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, tmp_leg_outside_margin])]
    rarm_vertices = rleg_vertices
    larm_vertices = lleg_vertices
    stp_org.eefm_support_polygon_vertices_sequence = map (lambda x : OpenHRP.AutoBalanceStabilizerService.SupportPolygonVertices(vertices=x), [lleg_vertices, rleg_vertices, larm_vertices, rarm_vertices])
    stp_org.eefm_leg_inside_margin=tmp_leg_inside_margin
    stp_org.eefm_leg_outside_margin=tmp_leg_outside_margin
    stp_org.eefm_leg_front_margin=tmp_leg_front_margin
    stp_org.eefm_leg_rear_margin=tmp_leg_rear_margin
    stp_org.eefm_k1=[-1.39899,-1.39899]
    stp_org.eefm_k2=[-0.386111,-0.386111]
    stp_org.eefm_k3=[-0.175068,-0.175068]
    stp_org.eefm_rot_damping_gain = [[20*1.6*1.5, 20*1.6*1.5, 1e5]]*4
    stp_org.eefm_pos_damping_gain = [[3500*50, 3500*50, 3500*1.0*1.5]]*4
    stp_org.eefm_swing_rot_damping_gain = stp_org.eefm_rot_damping_gain[0]
    stp_org.eefm_swing_pos_damping_gain = stp_org.eefm_pos_damping_gain[0]
    stp_org.eefm_use_swing_damping=True
    hcf.abst_svc.setStabilizerParam(stp_org)
    stp = hcf.abst_svc.getStabilizerParam()
    vcheck = stp.k_tpcc_p == stp_org.k_tpcc_p and stp.k_tpcc_x == stp_org.k_tpcc_x and stp.k_brot_p == stp_org.k_brot_p
    if vcheck:
        print >> sys.stderr, "  setStabilizerParam() => OK", vcheck
    assert(vcheck)

def changeContactDecisionThre (thre):
    stp = hcf.abst_svc.getStabilizerParam()
    stp.contact_decision_threshold=thre
    hcf.abst_svc.setStabilizerParam(stp)

def mimicInTheAir ():
    changeContactDecisionThre(10000) # [N]

def mimicOnTheGround ():
    changeContactDecisionThre(50) # [N], default

def checkActualBaseAttitude(thre=5.0): # degree
    '''Check whether the robot falls down based on actual robot base-link attitude.
    '''
    act_rpy = checkParameterFromLog("WAIST")[3:]
    ret = abs(math.degrees(act_rpy[0])) < thre and abs(math.degrees(act_rpy[1])) < thre
    print >> sys.stderr, "  ret = ", ret, ", actual base rpy = (", act_rpy, ")"
    return ret

def printActualBase():
    '''Print actual base pos and rot
    '''
    act_base = checkParameterFromLog("WAIST")
    print >> sys.stderr, "  actual base pos = ", act_base[0:3], "[m], actual base rpy = ", act_base[3:], "[rad]"

def changeSTAlgorithm (new_st_alg):
    stp = hcf.abst_svc.getStabilizerParam()
    if stp.st_algorithm != new_st_alg:
        hcf.abst_svc.stopStabilizer()
        stp.st_algorithm = new_st_alg
        hcf.abst_svc.setStabilizerParam(stp)
        hcf.abst_svc.startStabilizer ()
        # Wait for osscilation being samall
        hcf.setJointAngles(hcf.getJointAngles(), 2.0);
        hcf.waitInterpolation()

def demoSTLoadPattern ():
    print >> sys.stderr, "3. EEFMQP st + SequencePlayer loadPattern"
    if hcf.pdc:
        # Set initial pose of samplerobot-gopos000 before starting of ST
        hcf.seq_svc.setJointAnglesSequenceFull([[0.000242, -0.403476, -0.000185, 0.832071, -0.427767, -6.928952e-05, 0.31129, -0.159481, -0.115399, -0.636277, 0.0, 0.0, 0.0, 0.000242, -0.403469, -0.000185, 0.832073, -0.427775, -6.928781e-05, 0.31129, 0.159481, 0.115399, -0.636277, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], # jvss
                                               [[0]*29], # vels
                                               [[0]*29], # torques
                                               [[-0.014759, -4.336272e-05, 0.668138]], # poss
                                               [[-0.000245, -0.000862, 0.000171]], # rpys
                                               [[0]*3], # accs
                                               [[0.014052, 0.000203, -0.66798]], # zmps
                                               [[0]*6*4], # wrenchs
                                               [[1,1,0,0,1,1,1,1]], # optionals
                                               [0.5]); # tms
        hcf.abst_svc.stopAutoBalancer()
        hcf.seq_svc.waitInterpolation()
        stp = hcf.abst_svc.getStabilizerParam()
        stp.emergency_check_mode=OpenHRP.AutoBalanceStabilizerService.NO_CHECK # Disable checking of emergency error because currently this error checker does not work correctly during walking.
        hcf.abst_svc.setStabilizerParam(stp)
        #changeSTAlgorithm (OpenHRP.AutoBalanceStabilizerService.EEFMQPCOP)
        changeSTAlgorithm (OpenHRP.AutoBalanceStabilizerService.EEFMQP)
        hcf.abst_svc.startStabilizer()
        # Exec loadPattern
        HRPSYS_DIR=check_output(['pkg-config', 'hrpsys-base', '--variable=prefix']).rstrip()
        hcf.loadPattern(HRPSYS_DIR+'/share/hrpsys/samples/SampleRobot/data/samplerobot-gopos000', 0.0)
        hcf.waitInterpolation()
        ret = checkActualBaseAttitude()
        if ret:
            print >> sys.stderr, "  ST + loadPattern => OK"
        assert(ret)
    else:
        print >> sys.stderr, "  This sample is neglected in High-gain mode simulation"

def demoStartStopTPCCST ():
    print >> sys.stderr, "4. start and stop TPCC st"
    if hcf.pdc:
        # setup controllers
        hcf.abst_svc.startAutoBalancer(autobalancer_limbs)
        changeSTAlgorithm (OpenHRP.AutoBalanceStabilizerService.TPCC)
        hcf.abst_svc.startStabilizer ()
        ret = checkActualBaseAttitude()
        if ret:
            print >> sys.stderr, "  Start and Stop Stabilizer => OK"
        assert(ret)
    else:
        print >> sys.stderr, "  This sample is neglected in High-gain mode simulation"

def demoStartStopEEFMQPST ():
    print >> sys.stderr, "5. start and stop EEFMQP st"
    if hcf.pdc:
        # setup controllers
        hcf.abst_svc.startAutoBalancer(autobalancer_limbs)
        changeSTAlgorithm (OpenHRP.AutoBalanceStabilizerService.EEFMQP)
        hcf.abst_svc.startStabilizer()
        hcf.abst_svc.goPos(0.3, 0, 0)
        hcf.abst_svc.waitFootSteps()
        ret = checkActualBaseAttitude()
        if ret:
            print >> sys.stderr, "  Start and Stop Stabilizer => OK"
        assert(ret)
    else:
        print >> sys.stderr, "  This sample is neglected in High-gain mode simulation"

def demoSTStairWalk ():
    print >> sys.stderr, "6. EEFMQPCOP + stair"
    if hcf.pdc:
        # setup controllers
        printActualBase()
        changeSTAlgorithm (OpenHRP.AutoBalanceStabilizerService.EEFMQPCOP)
        hcf.abst_svc.startStabilizer()
        hcf.abst_svc.startAutoBalancer(autobalancer_limbs)
        hcf.seq_svc.setJointAngles(half_sitting_pose, 1.0);
        hcf.waitInterpolation();
        printActualBase()
        # set gg param
        ggp = hcf.abst_svc.getGaitGeneratorParam()[1]
        org_ggp = hcf.abst_svc.getGaitGeneratorParam()[1]
        ggp.default_orbit_type = OpenHRP.AutoBalanceStabilizerService.STAIR
        ggp.swing_trajectory_time_offset_xy2z=0.1
        ggp.swing_trajectory_delay_time_offset=0.2
        ggp.toe_heel_phase_ratio=[0.05, 0.25, 0.20, 0.0, 0.18, 0.23, 0.09]
        ggp.toe_pos_offset_x = 1e-3*182.0;
        ggp.heel_pos_offset_x = 1e-3*-72.0;
        ggp.toe_zmp_offset_x = 1e-3*182.0;
        ggp.heel_zmp_offset_x = 1e-3*-72.0;
        ggp.use_toe_heel_transition=True
        ggp.use_toe_heel_auto_set = True
        ggp.toe_angle = 20;
        ggp.heel_angle = 10;
        hcf.abst_svc.setGaitGeneratorParam(ggp)
        printActualBase()
        hcf.abst_svc.setFootSteps([OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0,-0.09,0], [1,0,0,0], "rleg")]),
                          OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.27,0.09,0.1], [1,0,0,0], "lleg")]),
                          OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.27,-0.09,0.1], [1,0,0,0], "rleg")]),
                          OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.54,0.09,0], [1,0,0,0], "lleg")]),
                          OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.54,-0.09,0], [1,0,0,0], "rleg")])], 0);
        printActualBase()
        hcf.abst_svc.waitFootSteps();
        printActualBase()
        # finish
        hcf.abst_svc.setGaitGeneratorParam(org_ggp)
        ret = checkActualBaseAttitude()
        printActualBase()
        if ret:
            print >> sys.stderr, "  ST + Stair => OK"
        assert(ret)
    else:
        print >> sys.stderr, "  This sample is neglected in High-gain mode simulation"

def demoSTToeHeelWalk ():
    print >> sys.stderr, "7. EEFMQPCOP + toeheel"
    if hcf.pdc:
        # setup controllers
        hcf.abst_svc.startAutoBalancer(autobalancer_limbs)
        hcf.co_svc.disableCollisionDetection()
        changeSTAlgorithm (OpenHRP.AutoBalanceStabilizerService.EEFMQPCOP)
        hcf.abst_svc.startStabilizer()
        hcf.seq_svc.setJointAngles(initial_pose, 2.0);
        hcf.waitInterpolation();
        # set gg param
        ggp = hcf.abst_svc.getGaitGeneratorParam()[1]
        org_ggp = hcf.abst_svc.getGaitGeneratorParam()[1]
        ggp.default_orbit_type = OpenHRP.AutoBalanceStabilizerService.RECTANGLE
        ggp.swing_trajectory_time_offset_xy2z=0.1
        ggp.swing_trajectory_delay_time_offset=0.2
        ggp.toe_heel_phase_ratio=[0.05, 0.35, 0.20, 0.0, 0.13, 0.13, 0.14]
        ggp.toe_pos_offset_x = 1e-3*182.0;
        ggp.heel_pos_offset_x = 1e-3*-72.0;
        ggp.toe_zmp_offset_x = 1e-3*182.0;
        ggp.heel_zmp_offset_x = 1e-3*-72.0;
        ggp.use_toe_heel_transition=True
        ggp.use_toe_heel_auto_set=True
        # test setFootStepsWithParam
        ggp.default_double_support_ratio=0.7
        hcf.abst_svc.setGaitGeneratorParam(ggp)
        for sgn in [1, -1]:
            hcf.abst_svc.setFootStepsWithParam([OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0,-0.09,0], [1,0,0,0], "rleg")]),
                                       OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([sgn*0.22,0.09,0], [1,0,0,0], "lleg")]),
                                       OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([sgn*0.44,-0.09,0], [1,0,0,0], "rleg")]),
                                       OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([sgn*0.44,0.09,0], [1,0,0,0], "lleg")])],
                                      [OpenHRP.AutoBalanceStabilizerService.StepParams([OpenHRP.AutoBalanceStabilizerService.StepParam(step_height=0.0, step_time=1.0, toe_angle=0.0, heel_angle=0.0)]),
                                       OpenHRP.AutoBalanceStabilizerService.StepParams([OpenHRP.AutoBalanceStabilizerService.StepParam(step_height=0.05, step_time=4.0, toe_angle=20.0, heel_angle=10.0)]),
                                       OpenHRP.AutoBalanceStabilizerService.StepParams([OpenHRP.AutoBalanceStabilizerService.StepParam(step_height=0.05, step_time=4.0, toe_angle=20.0, heel_angle=10.0)]),
                                       OpenHRP.AutoBalanceStabilizerService.StepParams([OpenHRP.AutoBalanceStabilizerService.StepParam(step_height=0.05, step_time=4.0, toe_angle=20.0, heel_angle=10.0)])], 0)
            hcf.abst_svc.waitFootSteps();
        # test goPos
        ggp.default_double_support_ratio=0.2
        ggp.stride_parameter=[0.22,0.1,20.0,0.22]
        ggp.toe_angle = 20;
        ggp.heel_angle = 10;
        hcf.abst_svc.setGaitGeneratorParam(ggp)
        for sgn in [1, -1]:
            hcf.abst_svc.goPos(sgn*0.66,sgn*0.2,sgn*40);
            hcf.abst_svc.waitFootSteps();
        # finish
        hcf.abst_svc.setGaitGeneratorParam(org_ggp)
        hcf.co_svc.enableCollisionDetection()
        ret = checkActualBaseAttitude()
        if ret:
            print >> sys.stderr, "  ST + ToeHeel => OK"
        assert(ret)
    else:
        print >> sys.stderr, "  This sample is neglected in High-gain mode simulation"

def demoSTTurnWalk ():
    print >> sys.stderr, "8. EEFMQPCOP st + Turn walk"
    if hcf.pdc:
        hcf.abst_svc.startAutoBalancer(autobalancer_limbs)
        hcf.co_svc.disableCollisionDetection()
        changeSTAlgorithm (OpenHRP.AutoBalanceStabilizerService.EEFMQPCOP)
        hcf.abst_svc.startStabilizer()
        ggp = hcf.abst_svc.getGaitGeneratorParam()[1]
        org_ggp = hcf.abst_svc.getGaitGeneratorParam()[1]
        ggp.stride_parameter=[0.15, 0.15, 90.0, 0.05]
        hcf.abst_svc.setGaitGeneratorParam(ggp)
        hcf.abst_svc.goPos(0,-0.2,0);
        hcf.abst_svc.waitFootSteps();
        hcf.abst_svc.goPos(0,0,175);
        hcf.abst_svc.waitFootSteps();
        hcf.abst_svc.goPos(0.4,0.15,40);
        hcf.abst_svc.waitFootSteps();
        # Wait for non-st osscilation being samalpl
        hcf.abst_svc.setGaitGeneratorParam(org_ggp)
        hcf.co_svc.enableCollisionDetection()
        ret = checkActualBaseAttitude()
        if ret:
            print >> sys.stderr, "  ST + Turnwalk => OK"
        assert(ret)
    else:
        print >> sys.stderr, "  This sample is neglected in High-gain mode simulation"


def demoSTTransitionAirGround ():
    # This example is from YoheiKakiuchi's comment : https://github.com/fkanehiro/hrpsys-base/issues/1098, https://github.com/fkanehiro/hrpsys-base/pull/1102#issuecomment-284609203
    print >> sys.stderr, "9. ST Transition (in the air and on the ground)"
    if hcf.pdc:
        # Init
        stp_org = hcf.abst_svc.getStabilizerParam()
        stp = hcf.abst_svc.getStabilizerParam()
        stp.transition_time = 0.1; # for fast checking
        hcf.abst_svc.setStabilizerParam(stp)
        # Tests
        print >> sys.stderr, "  9-1. Check in the air"
        hcf.abst_svc.startStabilizer()
        mimicInTheAir()
        hcf.setJointAngles(hcf.getJointAngles(), stp.transition_time);hcf.waitInterpolation() # Wait transition
        cmode1 = hcf.abst_svc.getStabilizerParam().controller_mode
        vcheck1 = (cmode1 == OpenHRP.AutoBalanceStabilizerService.MODE_AIR)
        print >> sys.stderr, "  9-2. Check on the ground"
        mimicOnTheGround()
        hcf.setJointAngles(hcf.getJointAngles(), stp.transition_time);hcf.waitInterpolation() # Wait transition
        cmode2 = hcf.abst_svc.getStabilizerParam().controller_mode
        vcheck2 = (cmode2 == OpenHRP.AutoBalanceStabilizerService.MODE_ST)
        print >> sys.stderr, "  9-3. Check in the air and then stopST"
        mimicInTheAir()
        hcf.setJointAngles(hcf.getJointAngles(), 0.01);hcf.waitInterpolation() # Wait until in the air flag is invoked in onExecute
        hcf.abst_svc.stopStabilizer()
        cmode3 = hcf.abst_svc.getStabilizerParam().controller_mode
        vcheck3 = (cmode3 == OpenHRP.AutoBalanceStabilizerService.MODE_IDLE)
        print >> sys.stderr, "  9-4. Check on the ground"
        mimicOnTheGround()
        hcf.setJointAngles(hcf.getJointAngles(), 0.01);hcf.waitInterpolation() # Wait until on the ground flag is invoked in onExecute
        hcf.abst_svc.startStabilizer()
        cmode4 = hcf.abst_svc.getStabilizerParam().controller_mode
        vcheck4 = (cmode4 == OpenHRP.AutoBalanceStabilizerService.MODE_ST)
        # Finsh
        hcf.abst_svc.setStabilizerParam(stp_org)
        vcheck_list = [vcheck1, vcheck2, vcheck3, vcheck4]
        print >> sys.stderr, "  ST Transition Air Ground vcheck = ", vcheck_list, ", cmode = ", [cmode1, cmode2, cmode3, cmode4]
        if all(vcheck_list):
            print >> sys.stderr, "  ST Transition Air Ground => OK"
        assert(all(vcheck_list))
    else:
        print >> sys.stderr, "  This sample is neglected in High-gain mode simulation"

def demoSTRootRotChange ():
    print >> sys.stderr, "10. ST root rot change"
    if hcf.pdc:
        # 10deg
        root_rot_x_pose=[-0.240857,-0.634561,0.012382,1.30211,-0.669201,0.073893,0.31129,-0.159481,-0.115399,-0.636277,0.0,0.0,0.0,-0.232865,-0.555515,0.011753,1.1356,-0.581653,0.06476,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.0,0.0,0.0]
        # 35deg
        root_rot_y_pose=[-1.706033e-05,-1.04708,-0.000479,0.497763,-0.060719,-0.000105,0.31129,-0.159481,-0.115399,-0.636277,0.0,0.0,0.0,-1.690260e-05,-1.04693,-0.000479,0.497318,-0.060422,-0.000105,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.0,0.0,0.0]
        # 25deg
        root_rot_z_pose=[-0.261382,-0.479591,-0.490714,1.26471,-0.722778,0.018041,0.31129,-0.159481,-0.115399,-0.636277,0.0,0.0,0.0,-0.313108,-0.610397,-0.535653,1.24943,-0.571839,-0.013257,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.0,0.0,0.0]
        # all 10deg
        root_rot_xyz_pose=[-0.378611,-0.81283,-0.238181,1.23534,-0.577915,0.061071,0.31129,-0.159481,-0.115399,-0.636277,0.0,0.0,0.0,-0.351695,-0.768514,-0.225097,1.05221,-0.442267,0.050849,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.0,0.0,0.0]
        hcf.abst_svc.startAutoBalancer(autobalancer_limbs);
        changeSTAlgorithm (OpenHRP.AutoBalanceStabilizerService.EEFMQPCOP)
        print >> sys.stderr, "  init"
        checkActualBaseAttitude()
        hcf.seq_svc.setJointAngles(root_rot_x_pose, 1.0);hcf.waitInterpolation();
        hcf.seq_svc.setJointAngles(root_rot_x_pose, 1.0);hcf.waitInterpolation(); # dummy for wait
        print >> sys.stderr, "  root rot x done."
        checkActualBaseAttitude()
        hcf.seq_svc.setJointAngles(root_rot_y_pose, 1.0);hcf.waitInterpolation();
        hcf.seq_svc.setJointAngles(root_rot_y_pose, 1.0);hcf.waitInterpolation(); # dummy for wait
        print >> sys.stderr, "  root rot y done."
        checkActualBaseAttitude()
        hcf.seq_svc.setJointAngles(root_rot_z_pose, 1.0);hcf.waitInterpolation();
        hcf.seq_svc.setJointAngles(root_rot_z_pose, 1.0);hcf.waitInterpolation(); # dummy for wait
        print >> sys.stderr, "  root rot z done."
        checkActualBaseAttitude()
        hcf.seq_svc.setJointAngles(root_rot_xyz_pose, 1.0);hcf.waitInterpolation();
        hcf.seq_svc.setJointAngles(root_rot_xyz_pose, 1.0);hcf.waitInterpolation(); # dummy for wait
        hcf.seq_svc.setJointAngles(initial_pose, 1.0);hcf.waitInterpolation();
        print >> sys.stderr, "  root rot xyz done."
        ret = checkActualBaseAttitude()
        if ret:
            print >> sys.stderr, "  ST root rot change => OK"
        assert(ret)
    else:
        print >> sys.stderr, "  This sample is neglected in High-gain mode simulation"

def demoSTMimicRouchTerrainWalk (terrain_height_diff = 0.04):
    print >> sys.stderr, "11. ST mimic rough terrain walk"
    if hcf.pdc:
        hcf.abst_svc.setFootSteps([OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0,-0.09,0], [1,0,0,0], "rleg")]),
                          OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.22,0.09,terrain_height_diff], [1,0,0,0], "lleg")]),
                          OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.44,-0.09,0], [1,0,0,0], "rleg")]),
                          OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.44,0.09,0], [1,0,0,0], "lleg")])], 0);
        hcf.abst_svc.waitFootSteps();
        hcf.abst_svc.setFootSteps([OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0,-0.09,0], [1,0,0,0], "rleg")]),
                          OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.22,0.09,-1*terrain_height_diff], [1,0,0,0], "lleg")]),
                          OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.44,-0.09,0], [1,0,0,0], "rleg")]),
                          OpenHRP.AutoBalanceStabilizerService.Footsteps([OpenHRP.AutoBalanceStabilizerService.Footstep([0.44,0.09,0], [1,0,0,0], "lleg")])], 0);
        hcf.abst_svc.waitFootSteps();
        ret = checkActualBaseAttitude()
        if ret:
            print >> sys.stderr, "  ST mimic rough terrain walk => OK"
        assert(ret)
    else:
        print >> sys.stderr, "  This sample is neglected in High-gain mode simulation"

def demoStabilizer():
    OPENHRP3_DIR=check_output(['pkg-config', 'openhrp3.1', '--variable=prefix']).rstrip()
    if os.path.exists(OPENHRP3_DIR+"/share/OpenHRP-3.1/sample/model/sample1_bush.wrl"):
        initStabilizer()
        demoGetStabilizerParam()
        demoSetStabilizerParam()
        demoSTLoadPattern()
        demoStartStopTPCCST()
        demoStartStopEEFMQPST()
        demoSTStairWalk()
        demoSTToeHeelWalk()
        demoSTTurnWalk()
        demoSTTransitionAirGround()
        demoSTRootRotChange()
        demoSTMimicRouchTerrainWalk()
    else:
        print >> sys.stderr, "Skip st test because of missing sample1_bush.wrl"

if __name__ == '__main__':
    start_time = time.time()
    initHcf()
    if StrictVersion(hrpsys_version) < StrictVersion('315.5.0'):
        sys.exit()
    if not hcf.pdc:
        demoAutoBalancer()
    else:
        print >> sys.stderr, "demoAutoBalancer is neglected in Torque-controll mode simulation"
    demoStabilizer()
    print >> sys.stderr, "All samplerobot_auto_balance_stabilzier.py demo time ", (time.time()-start_time), "[s]"
