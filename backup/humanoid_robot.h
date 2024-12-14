// humanoid_robot.h
#ifndef HUMANOID_ROBOT_H
#define HUMANOID_ROBOT_H

#include "primitive.h"
#include "odehandle.h"
#include "vsghandle.h"
#include <vector>
#include <memory>
#include <string>
#include <map>

namespace lpzrobots {

// Forward declarations
class JointMotor;
class RobotPart;

// Configuration structure for robot dimensions
struct RobotConfig {
    // Body dimensions
    double torsoWidth = 0.4;
    double torsoHeight = 0.6;
    double torsoDepth = 0.2;
    
    double headRadius = 0.15;
    
    double armRadius = 0.05;
    double armLength = 0.3;
    double forearmLength = 0.25;
    
    double legRadius = 0.06;
    double legLength = 0.4;
    double shinLength = 0.35;
    
    double footLength = 0.2;
    double footWidth = 0.1;
    double footHeight = 0.05;
    
    // Mass configurations
    double torsoMass = 2.0;
    double headMass = 0.5;
    double upperArmMass = 0.3;
    double forearmMass = 0.25;
    double upperLegMass = 0.5;
    double shinMass = 0.4;
    double footMass = 0.3;
    
    // Joint limits (in radians)
    double neckJointLimit = M_PI/4;
    double shoulderJointLimit = M_PI/2;
    double elbowJointLimit = 2*M_PI/3;
    double hipJointLimit = M_PI/2;
    double kneeJointLimit = 2*M_PI/3;
    double ankleJointLimit = M_PI/3;
};

// Class to manage a motor attached to a joint
class JointMotor {
public:
    JointMotor(dJointID joint, double maxForce, double maxVel);
    
    void setTargetPosition(double pos);
    void setTargetVelocity(double vel);
    void setPIDGains(double p, double i, double d);
    void update(double dt);
    
private:
    dJointID joint;
    int jointType;
    double maxForce;
    double maxVelocity;
    double targetPos;
    double targetVel;
    double kp, ki, kd;
    double integral;
    double lastError;
};

// Base class for robot parts
class RobotPart {
public:
    RobotPart(const std::string& name) : name(name) {}
    virtual ~RobotPart() = default;
    
    virtual void init(const OdeHandle& ode, const VsgHandle& vsg, 
                     const RobotConfig& config) = 0;
    virtual void update() = 0;
    
    const std::string& getName() const { return name; }
    Primitive* getPrimitive() const { return primitive.get(); }
    dBodyID getBody() const { return primitive ? primitive->getBody() : nullptr; }
    
protected:
    std::string name;
    std::unique_ptr<Primitive> primitive;
    std::vector<std::unique_ptr<JointMotor>> motors;
};

// Main humanoid robot class
class HumanoidRobot {
public:
    HumanoidRobot(const OdeHandle& ode, const VsgHandle& vsg, 
                  const RobotConfig& config = RobotConfig());
    ~HumanoidRobot();

    bool isInitialized() const { return initialized; }
    
    void init();
    void update(double dt);
    
    // High-level control methods
    void stand();
    void walk();
    void turnLeft();
    void turnRight();
    void raiseArms();
    void lowerArms();
    void updateWalkingMotion(double dt);
    
    // Get access to specific parts
    RobotPart* getPart(const std::string& name);
    
    // Set position of the entire robot
    void setPosition(const Pos& pos);
    std::vector<std::unique_ptr<JointMotor>> motors;
    
private:
    struct JointInfo {
        dJointID joint;
        std::unique_ptr<JointMotor> motor;
        std::string name;
        std::string parentName;
        std::string childName;
    };

    void createHingeJoint(const std::string& name,
                         const std::string& parent,
                         const std::string& child,
                         double limit,
                         const dVector3 axis);
    void cleanup();
    void updateJointPositions();
    void createParts();
    void createJoints();
    void setupMotors();
    void updateBalance(double dt);
    void positionParts();
    void createLimbJoints(const std::string& type, 
                         const std::string& childName,
                         const std::string& parentName,
                         double jointLimit);
    // Helper to set vector components
    void setVector3(dVector3 vec, double x, double y, double z) {
        vec[0] = x;
        vec[1] = y;
        vec[2] = z;
    }
    
    OdeHandle odeHandle;
    VsgHandle vsgHandle;
    RobotConfig config;

    std::vector<JointInfo> jointInfos;
    bool initialized;
    
    std::map<std::string, std::unique_ptr<RobotPart>> parts;
    std::vector<dJointID> joints;
    std::map<std::string, size_t> jointMap;
    
    bool isWalking;
    double walkPhase;
    bool cleanedUp;
};

} // namespace lpzrobots

#endif // HUMANOID_ROBOT_H
