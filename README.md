# Tocabi Avatar (Bipedal walking controller)

## Self-Collision Avoidance using Whole-Body Inverse Kinematics with Control Barrier Function

CamComJacobianWBIK (computeFast)

### Collision avoidance
- getSignedDistanceFunction()
- getSelfCollisionAvoidanceMatrix()

### Knee bend prevention
- getSignedDistanceFunction2()
- getKneeBendPreventionMatrix()
    - l_min and l_max are defined w.r.t link lengths and the knee joint angle boundaries. 