// humanoid_robot.cpp
#include "humanoid_robot.h"
#include <cmath>
#include <iostream>
#include <algorithm>
#include "color.h"

namespace lpzrobots {

const double DEG2RAD = M_PI / 180.0;
const double RAD2DEG = 180.0 / M_PI;

double clamp(double value, double min, double max) {
    return std::min(std::max(value, min), max);
}

// JointMotor Implementation
JointMotor::JointMotor(dJointID joint, double maxForce, double maxVel)
    : joint(joint), maxForce(maxForce), maxVelocity(maxVel),
      targetPos(0), targetVel(0), kp(200), ki(0.5), kd(20),
      integral(0), lastError(0) {
    if (!joint) {
        throw std::runtime_error("Invalid joint in JointMotor constructor");
    }
    // Store joint type for proper handling
    jointType = dJointGetType(joint);
}

void JointMotor::setTargetPosition(double pos) {
    if (!joint) return;  // Safety check
    targetPos = pos;
}

void JointMotor::setTargetVelocity(double vel) {
    targetVel = clamp(vel, -maxVelocity, maxVelocity);
}

void JointMotor::setPIDGains(double p, double i, double d) {
    kp = p;
    ki = i;
    kd = d;
}

void JointMotor::update(double dt) {
    if (!joint || dt <= 0) return;

    try {
        double currentPos = 0;
        double currentVel = 0;

        // Handle different joint types
        switch (jointType) {
            case dJointTypeHinge:
                currentPos = dJointGetHingeAngle(joint);
                currentVel = dJointGetHingeAngleRate(joint);
                break;
            case dJointTypeBall:
                // For ball joints, might need different control strategy
                return;
            default:
                return;
        }

        // Position error
        double error = targetPos - currentPos;
        
        // Anti-windup for integral term
        if (abs(error) < 0.1) {  // Only accumulate integral when near target
            integral = clamp(integral + error * dt, -maxForce/ki, maxForce/ki);
        } else {
            integral = 0;  // Reset integral when error is large
        }
        
        // Derivative with low-pass filter
        double derivative = (error - lastError) / dt;
        derivative = 0.9 * derivative;  // Simple low-pass filter
        
        // PID control with feed-forward velocity
        double force = kp * error +               // Proportional
                        ki * integral +              // Integral
                        kd * derivative +            // Derivative
                        targetVel * 0.1 * maxForce;  // Feed-forward term
        
        force = clamp(force, -maxForce, maxForce);
        
        // Apply force based on joint type
        switch (jointType) {
            case dJointTypeHinge:
                dJointAddHingeTorque(joint, force);
                break;
            default:
                break;
        }

        lastError = error;
    } catch (const std::exception& e) {
        std::cerr << "Error in JointMotor update: " << e.what() << std::endl;
    }
}

// Robot Part Implementations
class Torso : public RobotPart {
public:
    Torso() : RobotPart("torso") {}
    
    void init(const OdeHandle& ode, const VsgHandle& vsg, const RobotConfig& config) override {
        primitive = std::make_unique<Box>(config.torsoWidth, config.torsoHeight, config.torsoDepth);
        primitive->init(ode, config.torsoMass, vsg);
        primitive->setColor(Color(0.8, 0.8, 0.8));
    }
    
    void update() override {
        if (primitive) primitive->update();
    }
};

class Head : public RobotPart {
public:
    Head() : RobotPart("head") {}
    
    void init(const OdeHandle& ode, const VsgHandle& vsg, const RobotConfig& config) override {
        primitive = std::make_unique<Sphere>(config.headRadius);
        primitive->init(ode, config.headMass, vsg);
        primitive->setColor(Color(1.0, 0.8, 0.6));
    }
    
    void update() override {
        if (primitive) primitive->update();
    }
};

class Arm : public RobotPart {
public:
    Arm(const std::string& name) : RobotPart(name), isUpperArm(name.find("upper") != std::string::npos) {}
    
    void init(const OdeHandle& ode, const VsgHandle& vsg, const RobotConfig& config) override {
        double length = isUpperArm ? config.armLength : config.forearmLength;
        double mass = isUpperArm ? config.upperArmMass : config.forearmMass;
        
        primitive = std::make_unique<Capsule>(config.armRadius, length);
        primitive->init(ode, mass, vsg);
        primitive->setColor(Color(0.8, 0.6, 0.6));
    }
    
    void update() override {
        if (primitive) primitive->update();
    }
    
private:
    bool isUpperArm;
};

class Leg : public RobotPart {
public:
    Leg(const std::string& name) : RobotPart(name), isUpperLeg(name.find("upper") != std::string::npos) {}
    
    void init(const OdeHandle& ode, const VsgHandle& vsg, const RobotConfig& config) override {
        double length = isUpperLeg ? config.legLength : config.shinLength;
        double mass = isUpperLeg ? config.upperLegMass : config.shinMass;
        
        primitive = std::make_unique<Capsule>(config.legRadius, length);
        primitive->init(ode, mass, vsg);
        primitive->setColor(Color(0.6, 0.6, 0.8));
    }
    
    void update() override {
        if (primitive) primitive->update();
    }
    
private:
    bool isUpperLeg;
};

class Foot : public RobotPart {
public:
    Foot(const std::string& name) : RobotPart(name) {}
    
    void init(const OdeHandle& ode, const VsgHandle& vsg, const RobotConfig& config) override {
        primitive = std::make_unique<Box>(config.footLength, config.footWidth, config.footHeight);
        primitive->init(ode, config.footMass, vsg);
        primitive->setColor(Color(0.3, 0.3, 0.3));
    }
    
    void update() override {
        if (primitive) primitive->update();
    }
};

// Main HumanoidRobot Implementation
HumanoidRobot::HumanoidRobot(const OdeHandle& ode, const VsgHandle& vsg, const RobotConfig& config)
    : odeHandle(ode), vsgHandle(vsg), config(config),
      isWalking(false), walkPhase(0), initialized(false), cleanedUp(false) {
}

HumanoidRobot::~HumanoidRobot() {
    // Clean up joints explicitly since they're not managed by smart pointers
    if (!cleanedUp) {
        // Destroy joints here if they aren't already destroyed
        for (auto joint : joints) {
            if (joint) dJointDestroy(joint);
        }
    }
}

void HumanoidRobot::init() {
    std::cout << "Initializing humanoid robot..." << std::endl;

    // Verify/set joint limits
    config.elbowJointLimit = M_PI/2;   // 90 degrees
    // Increase joint limits for more visible motion
    config.hipJointLimit = M_PI/3;     // 60 degrees (was 45)
    config.kneeJointLimit = M_PI/2;    // 90 degrees (was 60)
    config.ankleJointLimit = M_PI/4;   // 45 degrees (was 30)
    config.shoulderJointLimit = M_PI/2; // 90 degrees
    
    std::cout << "Joint limits - Hip: " << config.hipJointLimit 
              << ", Knee: " << config.kneeJointLimit 
              << ", Ankle: " << config.ankleJointLimit << std::endl;
    
    try {
        std::cout << "Creating parts..." << std::endl;
        createParts();
        
        // Validate parts
        for (const auto& [name, part] : parts) {
            if (!part || !part->getPrimitive() || !part->getBody()) {
                throw std::runtime_error("Failed to initialize part: " + name);
            }
        }
        std::cout << "Parts created successfully" << std::endl;

        std::cout << "Creating joints..." << std::endl;
        createJoints();
        std::cout << "Joints created successfully" << std::endl;
        
        std::cout << "Setting up motors..." << std::endl;
        setupMotors();
        std::cout << "Motors set up successfully" << std::endl;

        std::cout << "Setting initial pose..." << std::endl;
        stand();
        
        initialized = true;
        std::cout << "Robot initialization complete" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error in robot init: " << e.what() << std::endl;
        cleanup();
        throw;
    }
}


void HumanoidRobot::cleanup() {
    if (!cleanedUp) {
        for (auto joint : joints) {
            if (joint) {
                dJointDestroy(joint);
                joint = nullptr;
            }
        }
        joints.clear();
        motors.clear();
        parts.clear();
        cleanedUp = true;
    }
}

void HumanoidRobot::updateJointPositions() {
    for (size_t i = 0; i < joints.size(); ++i) {
        if (!joints[i]) continue;

        dJointID joint = joints[i];
        
        // Handle different joint types
        switch (dJointGetType(joint)) {
            case dJointTypeBall: {
                dBodyID body1 = dJointGetBody(joint, 0);
                dBodyID body2 = dJointGetBody(joint, 1);
                if (body1 && body2) {
                    const dReal* pos1 = dBodyGetPosition(body1);
                    const dReal* pos2 = dBodyGetPosition(body2);
                    dJointSetBallAnchor(joint, 
                        (pos1[0] + pos2[0]) * 0.5,
                        (pos1[1] + pos2[1]) * 0.5,
                        (pos1[2] + pos2[2]) * 0.5);
                }
                break;
            }
            case dJointTypeHinge: {
                dBodyID body1 = dJointGetBody(joint, 0);
                dBodyID body2 = dJointGetBody(joint, 1);
                if (body1 && body2) {
                    const dReal* pos1 = dBodyGetPosition(body1);
                    const dReal* pos2 = dBodyGetPosition(body2);
                    dJointSetHingeAnchor(joint,
                        (pos1[0] + pos2[0]) * 0.5,
                        (pos1[1] + pos2[1]) * 0.5,
                        (pos1[2] + pos2[2]) * 0.5);
                }
                break;
            }
            default:
                // Handle other joint types if needed
                break;
        }
    }
}

void HumanoidRobot::createParts() {
    // Create torso first as it's the main body
    auto torso = std::make_unique<Torso>();
    torso->init(odeHandle, vsgHandle, config);
    parts["torso"] = std::move(torso);

    // Create head
    auto head = std::make_unique<Head>();
    head->init(odeHandle, vsgHandle, config);
    parts["head"] = std::move(head);

    // Create arms
    std::vector<std::string> armNames = {
        "left_upper_arm", "left_forearm",
        "right_upper_arm", "right_forearm"
    };
    
    for (const auto& name : armNames) {
        auto arm = std::make_unique<Arm>(name);
        arm->init(odeHandle, vsgHandle, config);
        parts[name] = std::move(arm);
    }

    // Create legs
    std::vector<std::string> legNames = {
        "left_upper_leg", "left_shin",
        "right_upper_leg", "right_shin"
    };
    
    for (const auto& name : legNames) {
        auto leg = std::make_unique<Leg>(name);
        leg->init(odeHandle, vsgHandle, config);
        parts[name] = std::move(leg);
    }

    // Create feet
    auto leftFoot = std::make_unique<Foot>("left_foot");
    leftFoot->init(odeHandle, vsgHandle, config);
    parts["left_foot"] = std::move(leftFoot);

    auto rightFoot = std::make_unique<Foot>("right_foot");
    rightFoot->init(odeHandle, vsgHandle, config);
    parts["right_foot"] = std::move(rightFoot);

    // Position all parts relative to torso
    positionParts();
}

void HumanoidRobot::positionParts() {
    auto& torso = parts["torso"];
    double torsoHeight = config.torsoHeight;
    double torsoDepth = config.torsoDepth;

    // Position head
    parts["head"]->getPrimitive()->setPosition(
        torso->getPrimitive()->getPosition() + 
        Pos(0, 0, torsoHeight/2 + config.headRadius));

    // Position arms
    double shoulderHeight = torsoHeight * 0.8;
    double shoulderWidth = config.torsoWidth/2 + config.armRadius;

    parts["left_upper_arm"]->getPrimitive()->setPosition(
        torso->getPrimitive()->getPosition() + 
        Pos(-shoulderWidth, 0, shoulderHeight));

    parts["right_upper_arm"]->getPrimitive()->setPosition(
        torso->getPrimitive()->getPosition() + 
        Pos(shoulderWidth, 0, shoulderHeight));

    // Position forearms
    double elbowHeight = shoulderHeight - config.armLength;
    parts["left_forearm"]->getPrimitive()->setPosition(
        torso->getPrimitive()->getPosition() + 
        Pos(-shoulderWidth, 0, elbowHeight));

    parts["right_forearm"]->getPrimitive()->setPosition(
        torso->getPrimitive()->getPosition() + 
        Pos(shoulderWidth, 0, elbowHeight));

    // Position legs
    double hipWidth = config.torsoWidth/3;
    double hipHeight = -torsoHeight/2;

    parts["left_upper_leg"]->getPrimitive()->setPosition(
        torso->getPrimitive()->getPosition() + 
        Pos(-hipWidth, 0, hipHeight));

    parts["right_upper_leg"]->getPrimitive()->setPosition(
        torso->getPrimitive()->getPosition() + 
        Pos(hipWidth, 0, hipHeight));

    // Position shins
    double kneeHeight = hipHeight - config.legLength;
    parts["left_shin"]->getPrimitive()->setPosition(
        torso->getPrimitive()->getPosition() + 
        Pos(-hipWidth, 0, kneeHeight));

    parts["right_shin"]->getPrimitive()->setPosition(
        torso->getPrimitive()->getPosition() + 
        Pos(hipWidth, 0, kneeHeight));

    // Position feet
    double ankleHeight = kneeHeight - config.shinLength;
    parts["left_foot"]->getPrimitive()->setPosition(
        torso->getPrimitive()->getPosition() + 
        Pos(-hipWidth, 0, ankleHeight - config.footHeight/2));

    parts["right_foot"]->getPrimitive()->setPosition(
        torso->getPrimitive()->getPosition() + 
        Pos(hipWidth, 0, ankleHeight - config.footHeight/2));
}

void HumanoidRobot::createJoints() {
    std::cout << "Creating joints..." << std::endl;
    joints.clear();  // Make sure we start fresh
    
    // Create neck joint (using ball joint for more freedom)
    dJointID neck = dJointCreateBall(odeHandle.world, 0);
    if (!neck) throw std::runtime_error("Failed to create neck joint");
    std::cout << "Created neck joint" << std::endl;
    
    dJointAttach(neck, parts["torso"]->getBody(), parts["head"]->getBody());
    const Pos& headPos = parts["head"]->getPrimitive()->getPosition();
    dJointSetBallAnchor(neck, headPos.x(), headPos.y(), headPos.z() - config.headRadius);
    joints.push_back(neck);
    std::cout << "Added neck joint. Total joints: " << joints.size() << std::endl;

    // Create shoulder joints
    dVector3 yAxis;
    setVector3(yAxis, 0, 1, 0);

    std::cout << "Creating shoulder joints..." << std::endl;
    createHingeJoint("left_shoulder", "torso", "left_upper_arm", config.shoulderJointLimit, yAxis);
    createHingeJoint("right_shoulder", "torso", "right_upper_arm", config.shoulderJointLimit, yAxis);

    std::cout << "Creating elbow joints..." << std::endl;
    createHingeJoint("left_elbow", "left_upper_arm", "left_forearm", config.elbowJointLimit, yAxis);
    createHingeJoint("right_elbow", "right_upper_arm", "right_forearm", config.elbowJointLimit, yAxis);

    std::cout << "Creating hip joints..." << std::endl;
    createHingeJoint("left_hip", "torso", "left_upper_leg", config.hipJointLimit, yAxis);
    createHingeJoint("right_hip", "torso", "right_upper_leg", config.hipJointLimit, yAxis);

    std::cout << "Creating knee joints..." << std::endl;
    createHingeJoint("left_knee", "left_upper_leg", "left_shin", config.kneeJointLimit, yAxis);
    createHingeJoint("right_knee", "right_upper_leg", "right_shin", config.kneeJointLimit, yAxis);

    std::cout << "Creating ankle joints..." << std::endl;
    createHingeJoint("left_ankle", "left_shin", "left_foot", config.ankleJointLimit, yAxis);
    createHingeJoint("right_ankle", "right_shin", "right_foot", config.ankleJointLimit, yAxis);

    std::cout << "Total joints created: " << joints.size() << std::endl;
    
    if (joints.size() != 11) {  // 1 neck + 4 arm + 6 leg joints
        throw std::runtime_error("Incorrect number of joints created. Expected 11, got " + 
                               std::to_string(joints.size()));
    }

    // Setup joint map
    jointMap.clear();
    jointMap["neck"] = 0;
    jointMap["left_shoulder"] = 1;
    jointMap["right_shoulder"] = 2;
    jointMap["left_elbow"] = 3;
    jointMap["right_elbow"] = 4;
    jointMap["left_hip"] = 5;
    jointMap["right_hip"] = 6;
    jointMap["left_knee"] = 7;
    jointMap["right_knee"] = 8;
    jointMap["left_ankle"] = 9;
    jointMap["right_ankle"] = 10;
}

void HumanoidRobot::createHingeJoint(const std::string& name,
                                    const std::string& parent,
                                    const std::string& child,
                                    double limit,
                                    const dVector3 axis) {
    if (!parts[parent] || !parts[child]) {
        throw std::runtime_error("Invalid parts for joint: " + name);
    }

    JointInfo jointInfo;
    jointInfo.name = name;
    jointInfo.parentName = parent;
    jointInfo.childName = child;

    // Create joint
    jointInfo.joint = dJointCreateHinge(odeHandle.world, 0);
    if (!jointInfo.joint) {
        throw std::runtime_error("Failed to create joint: " + name);
    }

    // Attach bodies
    dJointAttach(jointInfo.joint, 
                 parts[parent]->getBody(),
                 parts[child]->getBody());

    // Set joint anchor at the connection point between parts
    const Pos& childPos = parts[child]->getPrimitive()->getPosition();
    dJointSetHingeAnchor(jointInfo.joint, childPos.x(), childPos.y(), childPos.z());
    dJointSetHingeAxis(jointInfo.joint, axis[0], axis[1], axis[2]);

    // Set joint limits
    dJointSetHingeParam(jointInfo.joint, dParamLoStop, -limit);
    dJointSetHingeParam(jointInfo.joint, dParamHiStop, limit);
    dJointSetHingeParam(jointInfo.joint, dParamBounce, 0.0);
    dJointSetHingeParam(jointInfo.joint, dParamFMax, 1000.0);

    // Create motor with appropriate settings based on joint type
    double maxForce = 100.0;
    double maxVel = M_PI;
    
    if (name.find("ankle") != std::string::npos) {
        maxForce = 150.0;
        maxVel = M_PI/2;
    } else if (name.find("knee") != std::string::npos || 
               name.find("hip") != std::string::npos) {
        maxForce = 200.0;
        maxVel = M_PI/2;
    }

    try {
        jointInfo.motor = std::make_unique<JointMotor>(jointInfo.joint, maxForce, maxVel);
    } catch (const std::exception& e) {
        dJointDestroy(jointInfo.joint);
        throw;
    }

    jointInfos.push_back(std::move(jointInfo));
    // Add joint to joints vector - THIS WAS MISSING!
    joints.push_back(jointInfo.joint);
    
    std::cout << "Successfully created hinge joint: " << name 
              << " (total joints: " << joints.size() << ")" << std::endl;
}


void HumanoidRobot::createLimbJoints(const std::string& type, 
                                    const std::string& childName,
                                    const std::string& parentName,
                                    double jointLimit) {
    dJointID joint = dJointCreateUniversal(odeHandle.world, 0);
    dJointAttach(joint, parts[parentName]->getBody(), parts[childName]->getBody());
    
    // Set joint anchor point
    const Pos& childPos = parts[childName]->getPrimitive()->getPosition();
    dJointSetUniversalAnchor(joint, childPos.x(), childPos.y(), childPos.z());
    
    // Set joint axes based on type
    if (type == "shoulder" || type == "hip") {
        dJointSetUniversalAxis1(joint, 1, 0, 0); // Forward/backward rotation
        dJointSetUniversalAxis2(joint, 0, 1, 0); // Sideways rotation
    } else if (type == "elbow" || type == "knee") {
        dJointSetUniversalAxis1(joint, 1, 0, 0); // Main bending axis
        dJointSetUniversalAxis2(joint, 0, 1, 0); // Limited twist
    } else if (type == "ankle") {
        dJointSetUniversalAxis1(joint, 1, 0, 0); // Forward/backward rotation
        dJointSetUniversalAxis2(joint, 0, 1, 0); // Side-to-side rotation
    }
    
    // Set joint limits
    dJointSetUniversalParam(joint, dParamLoStop, -jointLimit);
    dJointSetUniversalParam(joint, dParamHiStop, jointLimit);
    dJointSetUniversalParam(joint, dParamLoStop2, -jointLimit/2);
    dJointSetUniversalParam(joint, dParamHiStop2, jointLimit/2);
    
    joints.push_back(joint);
}


void HumanoidRobot::setupMotors() {
    std::cout << "Setting up motors. Joint count: " << joints.size() << std::endl;
    
    if (joints.empty()) {
        throw std::runtime_error("No joints available for motor setup");
    }

    motors.clear();
    motors.reserve(joints.size());

    for (size_t i = 0; i < joints.size(); i++) {
        if (!joints[i]) {
            throw std::runtime_error("Null joint encountered at index " + std::to_string(i));
        }

        double maxForce = 100.0;
        double maxVel = M_PI * 2.0;
        
        if (i == 0) {  // Neck
            maxForce = 50.0;
            maxVel = M_PI;
            auto motor = std::make_unique<JointMotor>(joints[i], maxForce, maxVel);
            motor->setPIDGains(100.0, 0.1, 10.0);  // Soft control for neck
            motors.push_back(std::move(motor));
        } 
        else if (i >= 1 && i <= 4) {  // Arms
            maxForce = 150.0;  // Increased for better arm control
            maxVel = M_PI * 2.0;
            auto motor = std::make_unique<JointMotor>(joints[i], maxForce, maxVel);
            motor->setPIDGains(200.0, 0.2, 20.0);  // Medium stiffness for arms
            motors.push_back(std::move(motor));
        } 
        else if (i >= 5 && i <= 8) {  // Hips and knees
            maxForce = 300.0;  // Increased for better leg support
            maxVel = M_PI * 1.5;
            auto motor = std::make_unique<JointMotor>(joints[i], maxForce, maxVel);
            motor->setPIDGains(400.0, 0.4, 40.0);  // Stiff control for legs
            motors.push_back(std::move(motor));
        } 
        else {  // Ankles
            maxForce = 200.0;
            maxVel = M_PI;
            auto motor = std::make_unique<JointMotor>(joints[i], maxForce, maxVel);
            motor->setPIDGains(300.0, 0.3, 30.0);  // High stiffness for stability
            motors.push_back(std::move(motor));
        }
    }

    std::cout << "Motor setup complete. Total motors: " << motors.size() << std::endl;
}

void HumanoidRobot::update(double dt) {
    if (!initialized || dt <= 0) {
        std::cout << "Update skipped - not initialized or invalid dt: " << dt << std::endl;
        return;
    }

    static int updateCount = 0;
    updateCount++;
    
    // Only print every 60 updates to avoid spam
    if (updateCount % 60 == 0) {
        std::cout << "Update " << updateCount << " - dt: " << dt << std::endl;
    }

    try {
        if (isWalking) {
            updateWalkingMotion(dt);
        } 
        else {
            updateBalance(dt);
        }
        // updateBalance(dt);

        // Update all motors
        for (auto& motor : motors) {
            if (motor) {
                motor->update(dt);
            }
        }

        // Update all parts
        for (auto& [name, part] : parts) {
            if (part) {
                part->update();
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error in robot update: " << e.what() << std::endl;
    }
}

// void HumanoidRobot::update(double dt) {
//     if (!initialized || dt <= 0) return;

//     try {
//         updateBalance(dt);

//         if (isWalking) {
//             updateWalkingMotion(dt);
//         }

//         // Update all motors
//         for (auto& jointInfo : jointInfos) {
//             if (jointInfo.motor) {
//                 jointInfo.motor->update(dt);
//             }
//         }

//         // Update all parts
//         for (auto& [name, part] : parts) {
//             if (part) {
//                 part->update();
//             }
//         }
//     } catch (const std::exception& e) {
//         std::cerr << "Error in robot update: " << e.what() << std::endl;
//     }
// }

void HumanoidRobot::updateBalance(double dt) {
    if (!initialized || jointInfos.empty()) return;

    try {
        // Get center of mass
        dReal com[3] = {0, 0, 0};
        if (parts["torso"] && parts["torso"]->getBody()) {
            const dReal* torsoPos = dBodyGetPosition(parts["torso"]->getBody());
            for (int i = 0; i < 3; i++) com[i] = torsoPos[i];
        }

        // Calculate ankle angles for balance
        double ankleAngleX = -atan2(com[0], com[2]) * 0.5;
        double ankleAngleY = -atan2(com[1], com[2]) * 0.5;

        // Apply to ankle motors
        for (auto& jointInfo : jointInfos) {
            if (!jointInfo.motor) continue;
            
            if (jointInfo.name == "left_ankle" || jointInfo.name == "right_ankle") {
                jointInfo.motor->setTargetPosition(ankleAngleX);
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error in balance update: " << e.what() << std::endl;
    }
}

void HumanoidRobot::updateWalkingMotion(double dt) {
    if (!initialized || motors.size() < 11) return;

    // Slower, more stable walking motion
    walkPhase += dt * 2.0;  // Slower speed for stability
    if (walkPhase > 2 * M_PI) walkPhase -= 2 * M_PI;
    
    // Base walking pattern
    double hipAngle = sin(walkPhase) * (config.hipJointLimit * 0.6);  // Reduced range
    double kneeBase = config.kneeJointLimit * 0.3;  // Constant slight bend
    double kneeAngle = kneeBase + abs(sin(walkPhase - M_PI/4)) * (config.kneeJointLimit * 0.4);
    double ankleAngle = -sin(walkPhase + M_PI/6) * (config.ankleJointLimit * 0.4);
    
    try {
        // Constant slight forward lean for stability
        motors[0]->setTargetPosition(0.15);  // Slight forward lean
        
        // Left leg
        motors[5]->setTargetPosition(hipAngle);  // Left hip
        motors[7]->setTargetPosition(kneeAngle); // Left knee
        
        // Right leg (opposite phase)
        motors[6]->setTargetPosition(-hipAngle);  // Right hip
        motors[8]->setTargetPosition(kneeAngle);  // Right knee
        
        // Ankle control with stability compensation
        double leftAnkle = ankleAngle - hipAngle * 0.2;  // Compensate for hip motion
        double rightAnkle = -ankleAngle + hipAngle * 0.2;
        motors[9]->setTargetPosition(leftAnkle);   // Left ankle
        motors[10]->setTargetPosition(rightAnkle); // Right ankle
        
        // Natural arm swing (opposite to legs)
        double armAngle = -hipAngle * 0.7;  // Reduced arm swing
        motors[1]->setTargetPosition(armAngle);     // Left shoulder
        motors[2]->setTargetPosition(-armAngle);    // Right shoulder
        
        // Keep elbows slightly bent
        motors[3]->setTargetPosition(0.2);  // Left elbow
        motors[4]->setTargetPosition(0.2);  // Right elbow
        
    } catch (const std::exception& e) {
        std::cerr << "Error in walking motion update: " << e.what() << std::endl;
    }
}

void HumanoidRobot::stand() {
    isWalking = false;
    walkPhase = 0;
    
    // Reset all joint positions to standing pose
    for (size_t i = 0; i < motors.size(); i++) {
        if (i == 0) {  // Neck
            motors[i]->setTargetPosition(0);
        } else if (i >= 1 && i <= 4) {  // Arms
            motors[i]->setTargetPosition(0);
        } else if (i >= 5 && i <= 8) {  // Legs
            motors[i]->setTargetPosition(0);
        } else {  // Ankles
            motors[i]->setTargetPosition(0);
        }
    }
}

void HumanoidRobot::walk() {
    if (!initialized) {
        std::cerr << "Cannot walk: Robot not initialized" << std::endl;
        return;
    }
    
    if (motors.size() < 9) {
        std::cerr << "Cannot walk: Not enough motors (" << motors.size() << " motors)" << std::endl;
        return;
    }
    
    if (!isWalking) {
        std::cout << "Starting walk motion..." << std::endl;
        isWalking = true;
        walkPhase = 0;

        // Set initial pose for walking
        for (auto& motor : motors) {
            if (motor) {
                motor->setTargetPosition(0);  // Reset all joints
            }
        }
        
        // Slight knee bend for initial walking pose
        if (motors.size() > 8) {
            double initialKneeBend = 0.1;  // Small bend
            motors[7]->setTargetPosition(initialKneeBend);  // Left knee
            motors[8]->setTargetPosition(initialKneeBend);  // Right knee
        }
    }
}


void HumanoidRobot::turnLeft() {
    if (!parts["torso"] || !parts["torso"]->getBody()) return;
    
    dMatrix3 R;
    dRFromAxisAndAngle(R, 0, 0, 1, 0.1);  // Rotate around Z axis
    dBodySetRotation(parts["torso"]->getBody(), R);
}

void HumanoidRobot::turnRight() {
    if (!parts["torso"] || !parts["torso"]->getBody()) return;
    
    dMatrix3 R;
    dRFromAxisAndAngle(R, 0, 0, 1, -0.1);  // Rotate around Z axis
    dBodySetRotation(parts["torso"]->getBody(), R);
}

void HumanoidRobot::raiseArms() {
    // Set shoulder joints to raised position
    size_t leftShoulderIndex = 1;
    size_t rightShoulderIndex = 2;
    
    motors[leftShoulderIndex]->setTargetPosition(M_PI/2);
    motors[rightShoulderIndex]->setTargetPosition(M_PI/2);
}

void HumanoidRobot::lowerArms() {
    // Set shoulder joints to lowered position
    size_t leftShoulderIndex = 1;
    size_t rightShoulderIndex = 2;
    
    motors[leftShoulderIndex]->setTargetPosition(0);
    motors[rightShoulderIndex]->setTargetPosition(0);
}


void HumanoidRobot::setPosition(const Pos& pos) {
    if (!parts["torso"] || !parts["torso"]->getPrimitive()) {
        std::cerr << "Cannot set position: torso not initialized" << std::endl;
        return;
    }

    try {
        // Store initial positions relative to torso
        std::map<std::string, Pos> relativePositions;
        const Pos& oldTorsoPos = parts["torso"]->getPrimitive()->getPosition();
        
        for (const auto& [name, part] : parts) {
            if (part && part->getPrimitive()) {
                Pos partPos = part->getPrimitive()->getPosition();
                relativePositions[name] = partPos - oldTorsoPos;
            }
        }

        // Set new torso position
        parts["torso"]->getPrimitive()->setPosition(pos);

        // Update all other parts maintaining relative positions
        for (const auto& [name, part] : parts) {
            if (name != "torso" && part && part->getPrimitive()) {
                part->getPrimitive()->setPosition(pos + relativePositions[name]);
            }
        }

        // Carefully update joints and motors
        updateJointPositions();
        
    } catch (const std::exception& e) {
        std::cerr << "Error setting position: " << e.what() << std::endl;
    }
}

RobotPart* HumanoidRobot::getPart(const std::string& name) {
    auto it = parts.find(name);
    return it != parts.end() ? it->second.get() : nullptr;
}

}  // namespace lpzrobots