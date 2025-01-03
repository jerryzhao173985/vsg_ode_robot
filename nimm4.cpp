#include <assert.h>
#include <ode/ode.h>

// include primitives (box, spheres, cylinders ...)
#include "primitive.h"
#include "primitive.h"

// include joints
#include "joint.h"

// include header file
#include "nimm4.h"

namespace lpzrobots {

  // constructor:
  // - give handle for ODE and OSG stuff
  // - size of robot, maximal used force and speed factor are adjustable
  // - sphereWheels switches between spheres or wheels as wheels
  //   (wheels are only drawn, collision handling is always with spheres)
  Nimm4::Nimm4(const OdeHandle& odeHandle, const VsgHandle& vsgHandle,
               const std::string& name,
               double size/*=1.0*/, double force /*=3*/, double speed/*=15*/,
               bool sphereWheels /*=true*/)
    : // calling OdeRobots construtor with name of the actual robot
      OdeRobot(odeHandle, vsgHandle, name, "$Id$")
  {
    // robot is not created till now
    created=false;

    // choose color (here the color of the "Nimm Zwei" candy is used,
    // where the name of the Nimm2 and Nimm4 robots comes from ;-)
    this->vsgHandle.color = Color(2, 156/255.0, 0, 1.0f);

    // maximal used force is calculated from the force factor and size given to the constructor
    max_force   = force*size*size;

    // speed and type of wheels are set
    this->speed = speed;
    this->sphereWheels = sphereWheels;

    height=size;
    length=size/2.5; // length of body
    width=size/2;  // diameter of body
    radius=size/6; // wheel radius
    wheelthickness=size/20; // thickness of the wheels (if wheels used, no spheres)
    cmass=8*size;  // mass of the body
    wmass=size;    // mass of the wheels
    sensorno=4;    // number of sensors
    motorno=4;     // number of motors
    segmentsno=5;  // number of segments of the robot

    wheelsubstance.toRubber(50);

  };


  /** sets actual motorcommands
      @param motors motors scaled to [-1,1]
      @param motornumber length of the motor array
  */
  void Nimm4::setMotorsIntern(const double* motors, int motornumber){
    assert(created); // robot must exist
    // the number of controlled motors is minimum of
    // "number of motorcommands" (motornumber) and
    // "number of motors inside the robot" (motorno)
    int len = (motornumber < motorno)? motornumber : motorno;

    // for each motor the motorcommand (between -1 and 1) multiplied with speed
    // is set and the maximal force to realize this command are set
    for (int i=0; i<len; i++){
      joints[i]->setParam(dParamVel2, motors[i]*speed);
      joints[i]->setParam(dParamFMax2, max_force);
    }
  };

  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] (more or less)
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  int Nimm4::getSensorsIntern(sensor* sensors, int sensornumber){
    assert(created); // robot must exist

    // the number of sensors to read is the minimum of
    // "number of sensors requested" (sensornumber) and
    // "number of sensors inside the robot" (sensorno)
    int len = (sensornumber < sensorno)? sensornumber : sensorno;

    // for each sensor the anglerate of the joint is red and scaled with 1/speed
    for (int i=0; i<len; i++){
      sensors[i]=dynamic_cast<Hinge2Joint*>(joints[i])->getPosition2Rate();
      sensors[i]/=speed;  //scaling
    }
    // the number of red sensors is returned
    return len;
  };


  void Nimm4::placeIntern(const vsg::dmat4& pose){
    // the position of the robot is the center of the body (without wheels)
    // to set the vehicle on the ground when the z component of the position is 0
    // width*0.6 is added (without this the wheels and half of the robot will be in the ground)
    vsg::dmat4 p2;
    // The pose from user/API represents where they want the robot's reference point to be
    // By multiplying the translation first, "take the user's desired position (pose) and offset it by width*0.6 in Z"
    // when a user specifies pose as (0,0,0), the robot will actually be placed at (0,0,width*0.6)
    p2 = vsg::translate(vsg::dvec3(0.0, 0.0, width*0.6)) * pose;
    create(p2);
  };


  /**
   * updates the osg notes
   */
  void Nimm4::update() {
    OdeRobot::update();
    assert(created); // robot must exist

    for (int i=0; i<segmentsno; i++) { // update objects
      objects[i]->update();
    }
    for (int i=0; i < 4; i++) { // update joints
      joints[i]->update();
    }

  };


  /** creates vehicle at desired pose
      @param pose matrix with desired position and orientation
  */
  void Nimm4::create( const vsg::dmat4& pose ){
    if (created) {  // if robot exists destroy it
      destroy();
    }
    // create car space
    odeHandle.createNewSimpleSpace(parentspace, true);
    objects.resize(5);  // 1 capsule, 4 wheels
    joints.resize(4); // joints between cylinder and each wheel

    OdeHandle wheelHandle(odeHandle);
    // make the material of the wheels a hard rubber
    wheelHandle.substance = wheelsubstance;
    // create cylinder for main body
    // initialize it with ode-, osghandle and mass
    // rotate and place body (here by -90� around the y-axis)
    // use texture 'wood' for capsule
    // put it into objects[0]
    Capsule* cap = new Capsule(width/2, length);
    // cap->setTexture("Images/wood.rgb");
    cap->init(odeHandle, cmass, vsgHandle, Primitive::Body | Primitive::Geom | Primitive::Draw);
    std::cout << "Nimm4::create() - Capsule created" << std::endl;
    cap->setPose(pose * vsg::rotate(-M_PI/2, 0.0, 1.0, 0.0));
    objects[0]=cap;

    // create wheels
    /*   front
         -----
      1 |     | 2
        |     |
        |     |
      3 |     | 4
         -----
     */
    for (int i=1; i<5; i++) {
      // create sphere with radius
      // and initialize it with odehandle, osghandle and mass
      // calculate position of wheels(must be at desired positions relative to the body)
      // rotate and place body (here by 90Deg around the x-axis)
      // set texture for wheels
      Sphere* sph = new Sphere(radius);
      // sph->setTexture("Images/wood.rgb");
      sph->init(wheelHandle, wmass, vsgHandle.changeColor(Color(0.8,0.8,0.8)), Primitive::Body | Primitive::Geom | Primitive::Draw);
      // Calculate wheel positions relative to body
      vsg::dvec3 wpos = vsg::dvec3( ((i-1)/2==0?-1:1)*length/2.0,     // x: front/back
                        ((i-1)%2==0?-1:1)*(width*0.5+wheelthickness), // y: left/right
                        -width*0.6+radius ); // z: no additional offset needed since body is already positioned
      std::cout << "Nimm4::create() - Wheel " << i << " created" << std::endl;
      sph->setPose(pose * vsg::translate(wpos) * vsg::rotate(M_PI/2, 0.0, 0.0, 1.0));
      objects[i]=sph;
    }

    // generate 4 joints to connect the wheels to the body
    for (int i=0; i<4; i++) {
      Pos anchor(dBodyGetPosition(objects[i+1]->getBody()));
      std::cout << "Nimm4::create() - Anchor " << i << " created" << std::endl;
      joints[i] = new Hinge2Joint(objects[0], objects[i+1], anchor, pose * Axis::Z(), pose * Axis::Y());
      joints[i]->init(odeHandle, vsgHandle, true, 2.01 * radius);
    }
    for (int i=0; i<4; i++) {
      // set stops to make sure wheels always stay in alignment
      joints[i]->setParam(dParamLoStop, 0);
      joints[i]->setParam(dParamHiStop, 0);
    }

    created=true; // robot is created
  };


  /** destroys vehicle and space
   */
  void Nimm4::destroy(){
    if (created){
      cleanup();
      odeHandle.deleteSpace(); // destroy space
    }
    created=false; // robot does not exist (anymore)
  }

}
