#include "HumanoidRobot.h"
#include <ode/ode.h>
#include <vsg/all.h>

#include <cmath>
#include <iostream>

void HumanoidRobot::init(lpzrobots::OdeHandle& odeHandle, lpzrobots::VsgHandle& vsgHandle) {
    double mass = 1.0; // you can adjust masses for realism

    // Torso: center at (0,0,1.3)
    torso = new lpzrobots::Box(0.4, 0.2, 0.6);
    torso->init(odeHandle, mass*5, vsgHandle, lpzrobots::Primitive::Body|lpzrobots::Primitive::Geom|lpzrobots::Primitive::Draw);
    torso->setPose(lpzrobots::Pose(vsg::translate(0.0, 0.0, 1.3)));

    // Head: top of torso is at 1.3+(0.6/2)=1.6
    // Neck joint anchor at (0,0,1.6)
    // Head radius=0.12, head center at 1.6+0.12=1.72
    head = new lpzrobots::Sphere(0.12);
    head->init(odeHandle, mass, vsgHandle, lpzrobots::Primitive::Body|lpzrobots::Primitive::Geom|lpzrobots::Primitive::Draw);
    head->setPose(lpzrobots::Pose(vsg::translate(0.0, 0.0, 1.72)));
    addBallJoint(torso, head, vsg::dvec3(0.0, 0.0, 1.6), "neck", odeHandle, vsgHandle, showJoints, 0.1);

    // Arms:
    // Shoulder anchors at (±0.2,0,1.5)
    // Shoulders hinge about z-axis (0,0,1), so arms lift up/down at sides.
    // Upper arm length=0.3; place upper arm center at anchor-(0,0,0.15) since arm hangs down
    double armRadius=0.05; double armLength=0.3;

    leftUpperArm = new lpzrobots::Cylinder(armRadius, armLength);
    leftUpperArm->init(odeHandle, mass, vsgHandle, lpzrobots::Primitive::Body|lpzrobots::Primitive::Geom|lpzrobots::Primitive::Draw);
    leftUpperArm->setPose(lpzrobots::Pose(vsg::translate(-0.2,0,1.5-0.15)));

    addHingeJoint(torso, leftUpperArm, vsg::dvec3(-0.2,0,1.5), lpzrobots::Axis(0,0,1), "left_shoulder", odeHandle, vsgHandle, showJoints, 0.05);
    // Limit: arms can lift from -90 to +90 deg
    getJoint("left_shoulder")->setParam(dParamLoStop, -M_PI/2);
    getJoint("left_shoulder")->setParam(dParamHiStop, M_PI/2);

    rightUpperArm = new lpzrobots::Cylinder(armRadius, armLength);
    rightUpperArm->init(odeHandle, mass, vsgHandle, lpzrobots::Primitive::Body|lpzrobots::Primitive::Geom|lpzrobots::Primitive::Draw);
    rightUpperArm->setPose(lpzrobots::Pose(vsg::translate(0.2,0,1.5-0.15)));

    addHingeJoint(torso, rightUpperArm, vsg::dvec3(0.2,0,1.5), lpzrobots::Axis(0,0,1), "right_shoulder", odeHandle, vsgHandle, showJoints, 0.05);
    getJoint("right_shoulder")->setParam(dParamLoStop, -M_PI/2);
    getJoint("right_shoulder")->setParam(dParamHiStop, M_PI/2);

    // Elbows: at bottom of upper arm: (±0.2,0,1.5 -0.3=1.2)
    // Elbow hinges about y-axis (0,1,0), bending forearm forward
    leftLowerArm = new lpzrobots::Cylinder(armRadius, armLength);
    leftLowerArm->init(odeHandle, mass, vsgHandle, lpzrobots::Primitive::Body|lpzrobots::Primitive::Geom|lpzrobots::Primitive::Draw);
    leftLowerArm->setPose(lpzrobots::Pose(vsg::translate(-0.2,0,1.2-0.15)));

    addHingeJoint(leftUpperArm, leftLowerArm, vsg::dvec3(-0.2,0,1.2), lpzrobots::Axis(0,1,0), "left_elbow", odeHandle, vsgHandle, showJoints, 0.05);
    getJoint("left_elbow")->setParam(dParamLoStop, 0);
    getJoint("left_elbow")->setParam(dParamHiStop, M_PI/2);

    rightLowerArm = new lpzrobots::Cylinder(armRadius, armLength);
    rightLowerArm->init(odeHandle, mass, vsgHandle, lpzrobots::Primitive::Body|lpzrobots::Primitive::Geom|lpzrobots::Primitive::Draw);
    rightLowerArm->setPose(lpzrobots::Pose(vsg::translate(0.2,0,1.2-0.15)));

    addHingeJoint(rightUpperArm, rightLowerArm, vsg::dvec3(0.2,0,1.2), lpzrobots::Axis(0,1,0), "right_elbow", odeHandle, vsgHandle, showJoints, 0.05);
    getJoint("right_elbow")->setParam(dParamLoStop, 0);
    getJoint("right_elbow")->setParam(dParamHiStop, M_PI/2);

    // Legs:
    // Hips at (±0.1,0,1.0)
    // Universal joint: axis1=(0,1,0), axis2=(1,0,0)
    // Upper leg length=0.4, top at hip anchor, center at anchor-(0,0,0.2)
    double legRadius=0.06; double legLength=0.4;
    leftUpperLeg = new lpzrobots::Cylinder(legRadius, legLength);
    leftUpperLeg->init(odeHandle, mass*2, vsgHandle, lpzrobots::Primitive::Body|lpzrobots::Primitive::Geom|lpzrobots::Primitive::Draw);
    leftUpperLeg->setPose(lpzrobots::Pose(vsg::translate(-0.1,0,1.0-0.2)));

    addUniversalJoint(torso, leftUpperLeg, vsg::dvec3(-0.1,0,1.0), lpzrobots::Axis(0,1,0), lpzrobots::Axis(1,0,0), "left_hip", odeHandle, vsgHandle, showJoints, 0.05);
    getJoint("left_hip")->setParam(dParamLoStop, -M_PI/4);
    getJoint("left_hip")->setParam(dParamHiStop, M_PI/4);
    getJoint("left_hip")->setParam(dParamLoStop2, -M_PI/8);
    getJoint("left_hip")->setParam(dParamHiStop2, M_PI/8);

    rightUpperLeg = new lpzrobots::Cylinder(legRadius, legLength);
    rightUpperLeg->init(odeHandle, mass*2, vsgHandle, lpzrobots::Primitive::Body|lpzrobots::Primitive::Geom|lpzrobots::Primitive::Draw);
    rightUpperLeg->setPose(lpzrobots::Pose(vsg::translate(0.1,0,1.0-0.2)));

    addUniversalJoint(torso, rightUpperLeg, vsg::dvec3(0.1,0,1.0), lpzrobots::Axis(0,1,0), lpzrobots::Axis(1,0,0), "right_hip", odeHandle, vsgHandle, showJoints, 0.05);
    getJoint("right_hip")->setParam(dParamLoStop, -M_PI/4);
    getJoint("right_hip")->setParam(dParamHiStop, M_PI/4);
    getJoint("right_hip")->setParam(dParamLoStop2, -M_PI/8);
    getJoint("right_hip")->setParam(dParamHiStop2, M_PI/8);

    // Knees at bottom of upper leg: (±0.1,0,0.6)
    // Knees hinge about y-axis (0,1,0)
    leftLowerLeg = new lpzrobots::Cylinder(legRadius, legLength);
    leftLowerLeg->init(odeHandle, mass*2, vsgHandle, lpzrobots::Primitive::Body|lpzrobots::Primitive::Geom|lpzrobots::Primitive::Draw);
    leftLowerLeg->setPose(lpzrobots::Pose(vsg::translate(-0.1,0,0.6-0.2)));

    addHingeJoint(leftUpperLeg, leftLowerLeg, vsg::dvec3(-0.1,0,0.6), lpzrobots::Axis(0,1,0), "left_knee", odeHandle, vsgHandle, showJoints, 0.05);
    getJoint("left_knee")->setParam(dParamLoStop, 0);
    getJoint("left_knee")->setParam(dParamHiStop, M_PI/2);

    rightLowerLeg = new lpzrobots::Cylinder(legRadius, legLength);
    rightLowerLeg->init(odeHandle, mass*2, vsgHandle, lpzrobots::Primitive::Body|lpzrobots::Primitive::Geom|lpzrobots::Primitive::Draw);
    rightLowerLeg->setPose(lpzrobots::Pose(vsg::translate(0.1,0,0.6-0.2)));

    addHingeJoint(rightUpperLeg, rightLowerLeg, vsg::dvec3(0.1,0,0.6), lpzrobots::Axis(0,1,0), "right_knee", odeHandle, vsgHandle, showJoints, 0.05);
    getJoint("right_knee")->setParam(dParamLoStop, 0);
    getJoint("right_knee")->setParam(dParamHiStop, M_PI/2);
}

lpzrobots::HingeJoint* HumanoidRobot::addHingeJoint(lpzrobots::Primitive* part1, lpzrobots::Primitive* part2,
                                                    const vsg::dvec3& anchor, const lpzrobots::Axis& axis,
                                                    const std::string& name,
                                                    lpzrobots::OdeHandle& odeHandle, lpzrobots::VsgHandle& vsgHandle,
                                                    bool withVisual, double visualSize) {
    auto j = new lpzrobots::HingeJoint(part1, part2, anchor, axis);
    j->init(odeHandle, vsgHandle, withVisual, visualSize, true);
    joints.push_back(std::unique_ptr<lpzrobots::Joint>(j));
    jointMap[name] = j;
    return j;
}

lpzrobots::UniversalJoint* HumanoidRobot::addUniversalJoint(lpzrobots::Primitive* part1, lpzrobots::Primitive* part2,
                                                            const vsg::dvec3& anchor, const lpzrobots::Axis& axis1, const lpzrobots::Axis& axis2,
                                                            const std::string& name,
                                                            lpzrobots::OdeHandle& odeHandle, lpzrobots::VsgHandle& vsgHandle,
                                                            bool withVisual, double visualSize) {
    auto j = new lpzrobots::UniversalJoint(part1, part2, anchor, axis1, axis2);
    j->init(odeHandle, vsgHandle, withVisual, visualSize, true);
    joints.push_back(std::unique_ptr<lpzrobots::Joint>(j));
    jointMap[name] = j;
    return j;
}

lpzrobots::BallJoint* HumanoidRobot::addBallJoint(lpzrobots::Primitive* part1, lpzrobots::Primitive* part2,
                                                  const vsg::dvec3& anchor,
                                                  const std::string& name,
                                                  lpzrobots::OdeHandle& odeHandle, lpzrobots::VsgHandle& vsgHandle,
                                                  bool withVisual, double visualSize) {
    auto j = new lpzrobots::BallJoint(part1, part2, anchor);
    j->init(odeHandle, vsgHandle, withVisual, visualSize, true);
    joints.push_back(std::unique_ptr<lpzrobots::Joint>(j));
    jointMap[name] = j;
    return j;
}

void HumanoidRobot::update() {
    torso->update();
    head->update();
    leftUpperArm->update();
    leftLowerArm->update();
    rightUpperArm->update();
    rightLowerArm->update();
    leftUpperLeg->update();
    leftLowerLeg->update();
    rightUpperLeg->update();
    rightLowerLeg->update();

    for (auto &j : joints) {
        j->update();
    }
}

void HumanoidRobot::applyControl(double /*timeStep*/) {
    // Simple test: lift left arm upwards if angle <0.5, otherwise push down
    auto left_shoulder = getJoint("left_shoulder");
    if(left_shoulder) {
        auto hj = dynamic_cast<lpzrobots::HingeJoint*>(left_shoulder);
        if(hj) {
            double angle = hj->getPosition1();
            if(angle < 0.5) {
                hj->addForce1(1.0);
            } else {
                hj->addForce1(-1.0);
            }
        }
    }
}

void HumanoidRobot::setShowJoints(bool show) {
    showJoints = show;
    // If we wanted to dynamically toggle at runtime, we'd have to modify joint visuals (add/remove from scene).
    // For now, we assume they are only set at initialization.
}

lpzrobots::Joint* HumanoidRobot::getJoint(const std::string& name) {
    if (jointMap.find(name) != jointMap.end()) return jointMap[name];
    return nullptr;
}
