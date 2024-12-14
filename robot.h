#include <vector>
#include <memory>
#include <cassert>
#include <iostream>

#include "primitive.h"
#include "joint.h"
#include "axis.h"
#include "odehandle.h"
#include "vsghandle.h"

namespace lpzrobots {

  /**
   * A simple humanoid robot built from primitives and connected by joints.
   * This is a template for demonstration.
   *
   * Structure (approximate, not anatomically perfect, just for demonstration):
   *
   *          Head (Sphere)
   *          |
   *       (BallJoint)
   *          |
   *        Torso (Box)
   *       /         \
   * (Universal)   (Universal)
   *     /             \
   * UpperArm(L)      UpperArm(R)
   *    |                |
   *  (Hinge)          (Hinge)
   *    |                |
   * LowerArm(L)      LowerArm(R)
   *
   *    Below torso: (Universal) Joints at hips
   *          /                   \
   *      UpperLeg(L)           UpperLeg(R)
   *       |                      |
   *     (Hinge)                (Hinge)
   *       |                      |
   *    LowerLeg(L)            LowerLeg(R)
   *
   * Dimensions and positions are chosen so that the robot stands approximately upright.
   * The coordinate system: z is up, x forward, y sideways (or similar).
   *
   * You can enable/disable joint visuals by passing withVisual=false or true in init calls.
   */
  class HumanoidRobot {
  public:
    HumanoidRobot() = default;
    ~HumanoidRobot() {
      // Joints and primitives will be cleaned up by their shared_ptr destructors.
    }

    void buildRobot(const OdeHandle& odeHandle, const VsgHandle& vsgHandle, bool withVisual = true) {
      double density = false;  // we usually set absolute mass rather than density
      
      // Create Torso
      // A box roughly 0.3 m wide (y), 0.5 m deep (x), and 0.8 m tall (z)
      torso = std::make_shared<Box>(0.3, 0.5, 0.8);
      torso->init(odeHandle, /*mass*/10.0, vsgHandle, Primitive::Body|Primitive::Geom|Primitive::Draw);
      torso->setPose(Pose(vsg::translate(0.0, 0.0, 1.0))); // center torso at z=1.0

      // Create Head
      double headRadius = 0.15;
      head = std::make_shared<Sphere>(headRadius);
      head->init(odeHandle, /*mass*/1.0, vsgHandle, Primitive::Body|Primitive::Geom|Primitive::Draw);
      // place head above torso
      head->setPose(Pose(vsg::translate(0.0, 0.0, 1.8))); 

      // Head-Torso Joint (BallJoint)
      // anchor somewhere between head and torso
      vsg::dvec3 headAnchor(0.0, 0.0, 1.5);
      headTorsoJoint = std::make_shared<BallJoint>(torso.get(), head.get(), headAnchor);
      headTorsoJoint->init(odeHandle, vsgHandle, withVisual, 0.05, true);

      // Arms
      // UpperArm (Capsule): radius=0.05, length=0.4
      // We'll place shoulders at y ±0.2, about midway up torso at z=1.4
      double armRadius = 0.05;
      double upperArmLength = 0.4;
      double lowerArmLength = 0.4;

      // Left Upper Arm
      leftUpperArm = std::make_shared<Capsule>(armRadius, upperArmLength);
      leftUpperArm->init(odeHandle, /*mass*/1.5, vsgHandle, Primitive::Body|Primitive::Geom|Primitive::Draw);
      // place so that its center is at about (-0.15) in y from torso center
      leftUpperArm->setPose(Pose(vsg::translate(0.0, -0.25, 1.4))); 

      // Right Upper Arm
      rightUpperArm = std::make_shared<Capsule>(armRadius, upperArmLength);
      rightUpperArm->init(odeHandle, /*mass*/1.5, vsgHandle, Primitive::Body|Primitive::Geom|Primitive::Draw);
      rightUpperArm->setPose(Pose(vsg::translate(0.0, 0.25, 1.4))); 

      // Shoulder Joints (Universal or Ball joints). 
      // Let's use Universal joints for shoulders.
      // The anchor at shoulders: left at (0.0, -0.15, 1.4), right at (0.0, 0.15, 1.4)
      vsg::dvec3 leftShoulderAnchor(0.0, -0.15, 1.4);
      vsg::dvec3 rightShoulderAnchor(0.0, 0.15, 1.4);

      // Axes: For shoulder, let's say axis1 = z-axis, axis2 = x-axis for some rotation freedom
      Axis shoulderAxis1(0,0,1);
      Axis shoulderAxis2(1,0,0);

      leftShoulder = std::make_shared<UniversalJoint>(torso.get(), leftUpperArm.get(),
                                                      leftShoulderAnchor, shoulderAxis1, shoulderAxis2);
      leftShoulder->init(odeHandle, vsgHandle, withVisual, 0.05, true);

      rightShoulder = std::make_shared<UniversalJoint>(torso.get(), rightUpperArm.get(),
                                                       rightShoulderAnchor, shoulderAxis1, shoulderAxis2);
      rightShoulder->init(odeHandle, vsgHandle, withVisual, 0.05, true);


      // Lower Arms
      leftLowerArm = std::make_shared<Capsule>(armRadius, lowerArmLength);
      leftLowerArm->init(odeHandle, /*mass*/1.0, vsgHandle, Primitive::Body|Primitive::Geom|Primitive::Draw);
      leftLowerArm->setPose(Pose(vsg::translate(0.0, -0.25, 1.0))); // below upper arm

      rightLowerArm = std::make_shared<Capsule>(armRadius, lowerArmLength);
      rightLowerArm->init(odeHandle, /*mass*/1.0, vsgHandle, Primitive::Body|Primitive::Geom|Primitive::Draw);
      rightLowerArm->setPose(Pose(vsg::translate(0.0, 0.25, 1.0))); 

      // Elbow Joints (Hinge)
      // Anchor: roughly at (0.0, -0.25, 1.2) for left, and (0.0, 0.25, 1.2) for right
      // Axis: rotate around x-axis, for example.
      vsg::dvec3 leftElbowAnchor(0.0, -0.25, 1.2);
      vsg::dvec3 rightElbowAnchor(0.0, 0.25, 1.2);
      Axis elbowAxis(1,0,0);

      leftElbow = std::make_shared<HingeJoint>(leftUpperArm.get(), leftLowerArm.get(),
                                               leftElbowAnchor, elbowAxis);
      leftElbow->init(odeHandle, vsgHandle, withVisual, 0.05, true);

      rightElbow = std::make_shared<HingeJoint>(rightUpperArm.get(), rightLowerArm.get(),
                                                rightElbowAnchor, elbowAxis);
      rightElbow->init(odeHandle, vsgHandle, withVisual, 0.05, true);


      // Legs
      // Upper legs (Capsule): radius=0.07, length=0.4
      double legRadius = 0.07;
      double upperLegLength = 0.4;
      double lowerLegLength = 0.4;

      // Position hips at (0.0, ±0.1, 0.6) or so. Actually, torso bottom is at z=1.0 - 0.4=0.6 approx (half torso height)
      // Let's place upper legs starting slightly below the torso at z=0.6
      leftUpperLeg = std::make_shared<Capsule>(legRadius, upperLegLength);
      leftUpperLeg->init(odeHandle, /*mass*/2.0, vsgHandle, Primitive::Body|Primitive::Geom|Primitive::Draw);
      leftUpperLeg->setPose(Pose(vsg::translate(0.0, -0.1, 0.6)));

      rightUpperLeg = std::make_shared<Capsule>(legRadius, upperLegLength);
      rightUpperLeg->init(odeHandle, /*mass*/2.0, vsgHandle, Primitive::Body|Primitive::Geom|Primitive::Draw);
      rightUpperLeg->setPose(Pose(vsg::translate(0.0, 0.1, 0.6)));

      // Hip joints (Universal):
      // Anchor at hip: around (0.0, ±0.1, 1.0 - 0.4=0.6) 
      // Actually, let's align anchor where upper leg meets torso: at (0.0, ±0.1, 1.0 - 0.4=0.6)
      // that matches upper leg position we gave above.
      vsg::dvec3 leftHipAnchor(0.0, -0.1, 0.9); // slightly higher so leg can hang down
      vsg::dvec3 rightHipAnchor(0.0, 0.1, 0.9);

      // Axes for hips: 
      // axis1 = z-axis (turning leg forward/backward), axis2 = y-axis (abduction)
      Axis hipAxis1(0,0,1);
      Axis hipAxis2(0,1,0);

      leftHip = std::make_shared<UniversalJoint>(torso.get(), leftUpperLeg.get(),
                                                 leftHipAnchor, hipAxis1, hipAxis2);
      leftHip->init(odeHandle, vsgHandle, withVisual, 0.05, true);

      rightHip = std::make_shared<UniversalJoint>(torso.get(), rightUpperLeg.get(),
                                                  rightHipAnchor, hipAxis1, hipAxis2);
      rightHip->init(odeHandle, vsgHandle, withVisual, 0.05, true);

      // Lower Legs
      leftLowerLeg = std::make_shared<Capsule>(legRadius, lowerLegLength);
      leftLowerLeg->init(odeHandle, /*mass*/1.5, vsgHandle, Primitive::Body|Primitive::Geom|Primitive::Draw);
      leftLowerLeg->setPose(Pose(vsg::translate(0.0, -0.1, 0.2))); // below upper leg

      rightLowerLeg = std::make_shared<Capsule>(legRadius, lowerLegLength);
      rightLowerLeg->init(odeHandle, /*mass*/1.5, vsgHandle, Primitive::Body|Primitive::Geom|Primitive::Draw);
      rightLowerLeg->setPose(Pose(vsg::translate(0.0, 0.1, 0.2))); 

      // Knee Joints (Hinge)
      // Anchor about (0.0, ±0.1, 0.4) so that the leg can bend
      vsg::dvec3 leftKneeAnchor(0.0, -0.1, 0.4);
      vsg::dvec3 rightKneeAnchor(0.0, 0.1, 0.4);
      // Axis for knee: rotate around x-axis
      Axis kneeAxis(1,0,0);

      leftKnee = std::make_shared<HingeJoint>(leftUpperLeg.get(), leftLowerLeg.get(),
                                              leftKneeAnchor, kneeAxis);
      leftKnee->init(odeHandle, vsgHandle, withVisual, 0.05, true);

      rightKnee = std::make_shared<HingeJoint>(rightUpperLeg.get(), rightLowerLeg.get(),
                                               rightKneeAnchor, kneeAxis);
      rightKnee->init(odeHandle, vsgHandle, withVisual, 0.05, true);

      std::cout << "Humanoid robot built successfully." << std::endl;
    }

    // Call update on all joints and primitives
    void update() {
      // Update all primitives (though they might auto-update if integrated)
      torso->update();
      head->update();
      leftUpperArm->update(); leftLowerArm->update();
      rightUpperArm->update(); rightLowerArm->update();
      leftUpperLeg->update(); leftLowerLeg->update();
      rightUpperLeg->update(); rightLowerLeg->update();

      // Update joints
      headTorsoJoint->update();
      leftShoulder->update(); rightShoulder->update();
      leftElbow->update(); rightElbow->update();
      leftHip->update(); rightHip->update();
      leftKnee->update(); rightKnee->update();
    }

  private:
    // Primitives
    std::shared_ptr<Primitive> torso;
    std::shared_ptr<Primitive> head;

    std::shared_ptr<Primitive> leftUpperArm;
    std::shared_ptr<Primitive> leftLowerArm;
    std::shared_ptr<Primitive> rightUpperArm;
    std::shared_ptr<Primitive> rightLowerArm;

    std::shared_ptr<Primitive> leftUpperLeg;
    std::shared_ptr<Primitive> leftLowerLeg;
    std::shared_ptr<Primitive> rightUpperLeg;
    std::shared_ptr<Primitive> rightLowerLeg;

    // Joints
    std::shared_ptr<Joint> headTorsoJoint;

    std::shared_ptr<Joint> leftShoulder;
    std::shared_ptr<Joint> rightShoulder;
    std::shared_ptr<Joint> leftElbow;
    std::shared_ptr<Joint> rightElbow;

    std::shared_ptr<Joint> leftHip;
    std::shared_ptr<Joint> rightHip;
    std::shared_ptr<Joint> leftKnee;
    std::shared_ptr<Joint> rightKnee;
  };

} // namespace lpzrobots

/*
  Usage example (not a complete program):

  #include "odehandle.h"
  #include "vsghandle.h"

  int main() {
    lpzrobots::OdeHandle odeHandle; 
    lpzrobots::VsgHandle vsgHandle;

    // Setup ODE and VSG environment (not shown)
    // ...
    
    lpzrobots::HumanoidRobot humanoid;
    humanoid.buildRobot(odeHandle, vsgHandle, true);

    // In the simulation loop:
    // 1. Step ODE
    // 2. humanoid.update(); 
    // 3. Render VSG scene

    return 0;
  }

*/
