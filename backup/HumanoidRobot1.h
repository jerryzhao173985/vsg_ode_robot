#ifndef HUMANOID_ROBOT_H
#define HUMANOID_ROBOT_H

#include <vector>
#include <memory>
#include <map>
#include <string>
#include "primitive.h"
#include "joint.h"
#include "odehandle.h"
#include "vsghandle.h"
#include "axis.h"
#include "pos.h"
#include "pose.h"

class HumanoidRobot {
public:
    HumanoidRobot() : showJoints(true) {}

    void init(lpzrobots::OdeHandle& odeHandle, lpzrobots::VsgHandle& vsgHandle);
    void update();
    void applyControl(double timeStep);
    void setShowJoints(bool show);
    lpzrobots::Joint* getJoint(const std::string& name);

private:
    // Body parts
    lpzrobots::Primitive* torso;
    lpzrobots::Primitive* head;
    lpzrobots::Primitive* leftUpperArm;
    lpzrobots::Primitive* leftLowerArm;
    lpzrobots::Primitive* rightUpperArm;
    lpzrobots::Primitive* rightLowerArm;
    lpzrobots::Primitive* leftUpperLeg;
    lpzrobots::Primitive* leftLowerLeg;
    lpzrobots::Primitive* rightUpperLeg;
    lpzrobots::Primitive* rightLowerLeg;

    // Joints
    std::vector<std::unique_ptr<lpzrobots::Joint>> joints; 
    std::map<std::string, lpzrobots::Joint*> jointMap;

    bool showJoints;

    lpzrobots::HingeJoint* addHingeJoint(lpzrobots::Primitive* part1, lpzrobots::Primitive* part2,
                                         const vsg::dvec3& anchor, const lpzrobots::Axis& axis,
                                         const std::string& name,
                                         lpzrobots::OdeHandle& odeHandle, lpzrobots::VsgHandle& vsgHandle,
                                         bool withVisual, double visualSize);

    lpzrobots::UniversalJoint* addUniversalJoint(lpzrobots::Primitive* part1, lpzrobots::Primitive* part2,
                                                 const vsg::dvec3& anchor, const lpzrobots::Axis& axis1, const lpzrobots::Axis& axis2,
                                                 const std::string& name,
                                                 lpzrobots::OdeHandle& odeHandle, lpzrobots::VsgHandle& vsgHandle,
                                                 bool withVisual, double visualSize);

    lpzrobots::BallJoint* addBallJoint(lpzrobots::Primitive* part1, lpzrobots::Primitive* part2,
                                       const vsg::dvec3& anchor,
                                       const std::string& name,
                                       lpzrobots::OdeHandle& odeHandle, lpzrobots::VsgHandle& vsgHandle,
                                       bool withVisual, double visualSize);
};

#endif
