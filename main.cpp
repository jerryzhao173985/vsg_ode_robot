#include <vsg/all.h>
#include <ode/ode.h>
#include "odehandle.h"      // Your ODE handle
#include "primitive.h"   // Your VSG primitives
#include "vsghandle.h"   // Your VSG handle
#include "odeagent.h"    // Include OdeAgent definition
#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>
#include <memory>
#include <vector>
#include <cmath>
#include <sys/stat.h>
#include <unistd.h> // For getcwd
// #include "humanoid_robot.h"
// #include "robot.h"
#include "nimm2.h"
// #include "nimm4.h"
#include "fourwheeled.h"

// #include "sos.h"
#include "sox.h"

#include "one2onewiring.h"
#include "stl_adds.h"

using namespace lpzrobots;

double plane_z = 0.0;
GlobalData global;
bool drawContacts = false;

Plane* makePhysicsScene(OdeHandle& odeHandle, VsgHandle& vsgHandle){
    // add ODE Ground here (physical plane)
    dGeomID ground = dCreatePlane(odeHandle.space, 0 , 0 , 1 , plane_z);
    dGeomSetCategoryBits(ground,Primitive::Stat);
    dGeomSetCollideBits(ground,~Primitive::Stat);
    // assign a dummy primitive to the ground plane to have substance (material) support
    Plane* plane = new Plane();
    plane->init(odeHandle, 0.0, vsgHandle, Primitive::Geom | Primitive::Draw);
    plane->setPose(vsg::translate(0.0, 0.0, plane_z));
    dGeomSetData(ground, (void*)plane);
    //    std::cout << "GROUND: " << ground << std::endl;
    return plane;
}

// Function to add boundary walls to the scene
std::vector<Box*> addBoundaryWalls(OdeHandle& odeHandle, VsgHandle& vsgHandle, 
                                  double arenaSize = 20.0, double wallHeight = 3.0) {
    // Store all walls in a vector to return them
    std::vector<Box*> walls;
    
    // Wall thickness
    double wallThickness = 0.05;
    
    // Common options for all walls
    double mass = 0.0; // Static walls with no mass
    int mode = Primitive::Geom | Primitive::Draw; // Only need collision geometry and visual representation
    
    // Create a special VsgHandle with a different color for walls
    VsgHandle wallVsgHandle = vsgHandle;
    wallVsgHandle.color = Color(0.5, 0.5, 0.6); // Grey-blue walls
    
    // Define wall dimensions and positions
    struct WallDef {
        double lengthX, lengthY, lengthZ;
        vsg::dmat4 pose;
    };
    
    // Create 4 walls around the arena
    WallDef wallDefs[] = {
        // North wall (positive Y)
        { arenaSize*2 + wallThickness*2, wallThickness*2, wallHeight,
          vsg::translate(0.0, arenaSize/2 + wallThickness/2, plane_z) },
        // South wall (negative Y)
        { arenaSize*2 + wallThickness*2, wallThickness*2, wallHeight,
          vsg::translate(0.0, -arenaSize/2 - wallThickness/2, plane_z) },
        // East wall (positive X)
        { wallThickness*2, arenaSize*2 + wallThickness*2, wallHeight,
          vsg::translate(arenaSize/2 + wallThickness/2, 0.0, plane_z) },
        // West wall (negative X)
        { wallThickness*2, arenaSize*2 + wallThickness*2, wallHeight,
          vsg::translate(-arenaSize/2 - wallThickness/2, 0.0, plane_z) }
    };
    
    // Create each wall and add it to the scene
    for (const auto& def : wallDefs) {
        Box* wall = new Box(def.lengthX, def.lengthY, def.lengthZ);
        wall->init(odeHandle, mass, wallVsgHandle, mode);
        wall->setPose(def.pose);
        
        // Set wall substance to something hard
        Substance wallSubstance;
        wallSubstance.toPlastic(40); // Hard plastic
        wall->setSubstance(wallSubstance);
        
        walls.push_back(wall);
    }
    
    std::cout << "Created " << walls.size() << " boundary walls" << std::endl;
    return walls;
}

//------------------------------------------------------------------------
// DEBUGGING FUNCTIONS
//------------------------------------------------------------------------
bool isPlaceableGeom(dGeomID geom) {
    int class_id = dGeomGetClass(geom);
    return class_id != dPlaneClass && class_id != dRayClass;
}

const char* getGeomTypeName(dGeomID geom) {
    switch(dGeomGetClass(geom)) {
        case dSphereClass:      return "Sphere";
        case dBoxClass:         return "Box";
        case dCapsuleClass:     return "Capsule";
        case dCylinderClass:    return "Cylinder";
        case dPlaneClass:       return "Plane";
        case dRayClass:         return "Ray";
        case dTriMeshClass:     return "TriMesh";
        case dHeightfieldClass: return "Heightfield";
        default:                return "Unknown";
    }
}

void printGeomCollisionInfo(std::ostream& out, dGeomID o1, dGeomID o2, OdeHandle* handle) {
    out << "\n=== Collision Information ===\n";

    if (!o1 || !o2) {
        out << "Invalid geom detected!\n"
            << "o1: " << (void*)o1 << "\n"
            << "o2: " << (void*)o2 << "\n";
        return;
    }

    auto printGeomDetails = [&out](dGeomID geom, const char* prefix) {
        try {
            out << prefix << " Details:\n";
            out << "  Address: " << (void*)geom << "\n";
            out << "  Type: " << getGeomTypeName(geom) << "\n";
            
            bool isSpace = dGeomIsSpace(geom);
            out << "  Is Space: " << (isSpace ? "yes" : "no") << "\n";
            
            if (!isSpace) {
                bool placeable = isPlaceableGeom(geom);
                out << "  Is Placeable: " << (placeable ? "yes" : "no") << "\n";
                
                if (placeable) {
                    const dReal* pos = dGeomGetPosition(geom);
                    if (pos) {
                        out << "  Position: (" << pos[0] << ", " << pos[1] << ", " << pos[2] << ")\n";
                    }

                    dBodyID body = dGeomGetBody(geom);
                    if (body) {
                        const dReal* vel = dBodyGetLinearVel(body);
                        out << "  Has Body: yes\n";
                        if (vel) {
                            out << "  Body Velocity: (" << vel[0] << ", " << vel[1] << ", " << vel[2] << ")\n";
                        }
                    } else {
                        out << "  Has Body: no\n";
                    }
                }

                // For planes, print normal and distance
                if (dGeomGetClass(geom) == dPlaneClass) {
                    dVector4 params;
                    dGeomPlaneGetParams(geom, params);
                    out << "  Plane Normal: (" << params[0] << ", " << params[1] << ", " << params[2] << ")\n";
                    out << "  Plane Distance: " << params[3] << "\n";
                }
            }

            void* userData = dGeomGetData(geom);
            if (userData) {
                Primitive* prim = dynamic_cast<Primitive*>((Primitive*)userData);
                out << "  Has Primitive data: " << (prim ? "yes" : "no") << "\n";
            } else {
                out << "  Has Primitive data: no\n";
            }

        } catch (const std::exception& e) {
            out << "  ERROR: Exception while printing geom details: " << e.what() << "\n";
        } catch (...) {
            out << "  ERROR: Unknown exception while printing geom details\n";
        }
    };

    printGeomDetails(o1, "Geom1");
    printGeomDetails(o2, "Geom2");

    if (handle && handle->ignoredPairs) {
        out << "\nIgnored Pairs Status:\n"
            << "  Total ignored pairs: " << handle->ignoredPairs->size() << "\n"
            << "  This pair ignored: " << (handle->isIgnoredPair(o1, o2) ? "yes" : "no") << "\n";
    }

    out << "===========================\n";
}
//------------------------------------------------------------------------
// END DEBUGGING FUNCTIONS
//------------------------------------------------------------------------

void nearCallback(void *data, dGeomID o1, dGeomID o2) {
    if (!data || !o1 || !o2) return;  // Basic null checks
    
    if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {
        // colliding a space with something
        dSpaceCollide2(o1, o2, data, &nearCallback);
        // The collision of the geoms internal to the space(s)
        // is done separately in odeStep() (for each space that is not ignored once)
        return;
    }

    // Check for ignored pairs first
    if (((OdeHandle*)data)->isIgnoredPair(o1, o2)) {
        return;
    }

    // Get primitive data safely
    Primitive* p1 = dynamic_cast<Primitive*>((Primitive*)dGeomGetData(o1));
    Primitive* p2 = dynamic_cast<Primitive*>((Primitive*)dGeomGetData(o2));

    // If either primitive is missing, use default substance parameters
    static const Substance defaultSubstance;  // Using default constructor for safe defaults
    
    // Only allow a maximum of 10 contact points to avoid instability with too many constraints
    const int MAX_CONTACTS = 10;
    dContact contact[MAX_CONTACTS];
    int n = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));
    
    if (n > 0) {
        // Use default substance if primitive is missing
        const Substance& s1 = p1 ? p1->substance : defaultSubstance;
        const Substance& s2 = p2 ? p2->substance : defaultSubstance;
        
        dSurfaceParameters surfParams;
        int callbackrv = 1;

        // Only call callbacks if primitives exist
        if (p1 && s1.callback) {
            callbackrv = s1.callback(surfParams, global, s1.userdata, contact, n,
                                   o1, o2, s1, s2);
        }
        
        if (p2 && s2.callback && callbackrv == 1) {
            callbackrv = s2.callback(surfParams, global, s2.userdata, contact, n,
                                   o2, o1, s2, s1);
        }
        
        if (callbackrv == 1) {
            Substance::getSurfaceParams(surfParams, s1, s2, global.odeConfig.simStepSize);
            
            // Enhanced contact parameters for better sensor-obstacle interaction
            surfParams.bounce = 0.05;      // Minimal bounce for realistic contact
            surfParams.bounce_vel = 0.05;  // Low velocity threshold for bounce
            surfParams.soft_cfm = 0.008;   // Optimized for sensor feedback
            surfParams.soft_erp = 0.25;    // Good error reduction for stable contact
            
            // Add friction for better robot-obstacle interaction
            surfParams.mu = 0.8;           // Good friction coefficient
            surfParams.mu2 = 0.8;          // Equal friction in all directions
        }
        
        if (callbackrv == 0) return;

        // Create contact joints, limiting to MAX_CONTACTS
        for (int i = 0; i < n; ++i) {
            contact[i].surface = surfParams;
            dJointID c = dJointCreateContact(((OdeHandle*)data)->world,
                       ((OdeHandle*)data)->jointGroup,
                       &contact[i]);
            dJointAttach(c, dGeomGetBody(contact[i].geom.g1),
                           dGeomGetBody(contact[i].geom.g2));
        }

        // Draw contact points if requested
        if (drawContacts) {
            for (int i = 0; i < n; ++i) {
                global.addTmpObject(new TmpDisplayItem(new Box(0.02, 0.02, 0.02),
                                                     vsg::translate(Pos(contact[i].geom.pos)),
                                                     Color(1.0, 0, 0)),
                                  0.5);
            }
        }
    }
}

// This function is called, if there was a possible Collision detected (in a space used at call of dSpaceCollide (0))
void nearCallback_TopLevel(void *data, dGeomID o1, dGeomID o2) {

    bool collision_treated=false;
    // call robots collision treatments (old stuff, should be removed at some point)
    for(OdeAgentList::iterator i= global.agents.begin();
        (i != global.agents.end()) && !collision_treated; ++i) {
        collision_treated=(*i)->getRobot()->collisionCallback(data, o1, o2);
    }

    if (collision_treated)
        return; // exit if collision was treated by a robot

    nearCallback(data, o1, o2);
}

void odeStep(OdeHandle& odeHandle, double timestep){
    // Perform collision detection in the main space
    dSpaceCollide(odeHandle.space, &odeHandle, &nearCallback_TopLevel);
    
    // Also check collision in subspaces
    FOREACHC(std::vector<dSpaceID>, odeHandle.getSpaces(), i) {
        dSpaceCollide(*i, &odeHandle, &nearCallback);
    }
    
    // Adjust physics parameters for stability if needed for this step
    // Use a smaller step size if needed for stability
    const int numSubsteps = 2; // Use 2 substeps for more stable integration
    double subTimestep = timestep / numSubsteps;
    
    // Perform multiple smaller steps for better stability
    for (int i = 0; i < numSubsteps; i++) {
        dWorldQuickStep(odeHandle.world, subTimestep);
    }
    
    // Clear all contacts - important to do this AFTER all physics steps
    dJointGroupEmpty(odeHandle.jointGroup);

    // Update simulation time
    *odeHandle.time += timestep;
}

void simulateStep(OdeHandle& odeHandle, double timestep) {
    // Define some substeps for more stable integration
    const int numSubsteps = 2;
    const double subTimestep = timestep / numSubsteps;
    
    for (int i = 0; i < numSubsteps; i++) {
        // Clear contacts before collision detection
        dJointGroupEmpty(odeHandle.jointGroup);
        
        // Perform collision detection
        dSpaceCollide(odeHandle.space, &odeHandle, &nearCallback);
        
        // Advance physics simulation with smaller timestep
        dWorldQuickStep(odeHandle.world, subTimestep);
    }
    
    // Update simulation time
    *odeHandle.time += timestep;
}

// Function to display sensor information for debugging and monitoring
void displaySensorInfo(const OdeAgent* agent, int frameCount) {
    // Display sensor info every 100 frames to avoid spam
    if (frameCount % 100 == 0 && agent && agent->getRobot()) {
        const OdeRobot* robot = agent->getRobot();
        int sensorCount = robot->getSensorNumber();
        
        if (sensorCount > 4) {  // More than just wheel sensors
            std::cout << "\n=== Sensor-Equipped Robot Status (Frame " << frameCount << ") ===" << std::endl;
            std::cout << "Robot: " << robot->getName() << std::endl;
            std::cout << "Total sensors: " << sensorCount << std::endl;
            std::cout << "Position: "; 
            robot->getPosition().print();
            std::cout << "Expected sensors: 4 wheel sensors + IR sensors (front, back, side)" << std::endl;
            std::cout << "IR sensors should detect obstacles within range for navigation" << std::endl;
            std::cout << "======================================================\n" << std::endl;
        }
    }
}

void createNewDir(const char* base, char *newdir) {
    struct stat s;
    char fullpath[1024];
    
    for(int i=0; i<1000; ++i) {
        if(i==0)
            sprintf(newdir, "%s", base);
        else
            sprintf(newdir, "%s%03i", base, i);
            
        // Check if directory exists
        if(stat(newdir, &s) != 0) { 
            // Directory doesn't exist, create it
            #ifdef _WIN32
                int result = mkdir(newdir);
            #else
                int result = mkdir(newdir, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
            #endif
            
            if(result != 0) {
                std::cerr << "Error creating directory: " << newdir 
                          << " (errno: " << errno << ")" << std::endl;
                perror("mkdir failed");
                
                // Try with absolute path
                char cwd[1024];
                if(getcwd(cwd, sizeof(cwd)) != NULL) {
                    sprintf(fullpath, "%s/%s", cwd, newdir);
                    std::cout << "Trying with absolute path: " << fullpath << std::endl;
                    
                    #ifdef _WIN32
                        result = mkdir(fullpath);
                    #else
                        result = mkdir(fullpath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
                    #endif
                    
                    if(result == 0) {
                        strcpy(newdir, fullpath);
                        std::cout << "Successfully created directory: " << newdir << std::endl;
                        return;
                    } else {
                        std::cerr << "Failed to create directory with absolute path: " 
                                  << fullpath << std::endl;
                        perror("mkdir failed with absolute path");
                    }
                }
            } else {
                std::cout << "Successfully created directory: " << newdir << std::endl;
                return;
            }
        } else if(S_ISDIR(s.st_mode)) {
            // Directory exists
            std::cout << "Using existing directory: " << newdir << std::endl;
            return;
        }
        // If we get here, either the path exists but is not a directory,
        // or directory creation failed. Try the next index.
    }
    
    std::cerr << "ERROR: Failed to create directory after 1000 attempts." << std::endl;
    // Fall back to current directory
    strcpy(newdir, "./");
}


// predicate that matches agents that have the same name prefix
struct agent_match_prefix {
    typedef const OdeAgent* argument_type;
    typedef bool result_type;
    agent_match_prefix(std::string nameprefix)
      : nameprefix(nameprefix) {
      len=nameprefix.length();
    }
    bool operator()(const OdeAgent* a) {
      if(!a || !a->getRobot()) return false;
      return nameprefix.compare(0,len, a->getRobot()->getName(),0,len) == 0;
    }

    std::string nameprefix;
    int len;
};


OdeAgent* createVehicle(const OdeHandle& odeHandle, const VsgHandle& vsgHandle,
                              GlobalData& global, vsg::dmat4 pose, int type){
    Nimm2Conf nimm2conf = Nimm2::getDefaultConf();
    std::string name;
    switch(type){
    case 0:
        name = "CylinderVehicle";
        break;
    case 1:
        name = "BoxVehicle";
        nimm2conf.size = 1;
        nimm2conf.force = 5;
        nimm2conf.speed=20;
        nimm2conf.singleMotor=false;
        nimm2conf.boxMode=true;
        nimm2conf.boxWidth=1.5;
        break;
    case 2:
        name = "SphereVehicle";
        nimm2conf.size = 1;
        nimm2conf.force = 5;
        nimm2conf.speed=20;
        nimm2conf.singleMotor=false;
        nimm2conf.sphereWheels=true;
        nimm2conf.wheelSize=1;
        nimm2conf.wheelOffset=-1.0; // disabled
        nimm2conf.wheelSlip=0;
        break;
    default:
        // find robot and do naming
        name = "LongVehicle";
        nimm2conf.size = 1;
        nimm2conf.force = 5;
        nimm2conf.speed=20;
        //      nimm2conf.speed=15;
        nimm2conf.cigarMode=true;
        nimm2conf.cigarLength= 3.0;
        nimm2conf.singleMotor=false;
        nimm2conf.boxMode=true;
        nimm2conf.boxWidth=1.5;
        //      nimm2conf.visForce =true;
        nimm2conf.bumper=true;
    }

    int num = count_if(global.agents.begin(), global.agents.end(), agent_match_prefix(name));
    name += "_" + std::to_string(num+1);
    
    // Ensure robot is placed above the ground plane to avoid initial penetration
    // For type 4 (FourWheeled), place it higher to avoid initial instability
    double initialHeight = 0.1; // Small clearance above ground
    
    if(type == 4){
        FourWheeledConf conf = FourWheeled::getDefaultConf();
        // Ensure better stability with more appropriate parameters
        conf.force = 5.0;
        conf.speed = 15.0;
        conf.size = 1.0;
        
        // Enable IR sensors for obstacle detection and navigation
        conf.irFront = true;        // Front sensors for forward obstacle detection
        conf.irBack = true;         // Rear sensors for backward movement
        conf.irSide = true;         // Side sensors for comprehensive environment awareness
        conf.irRangeFront = 3.0;    // Extended range for forward planning
        conf.irRangeBack = 2.0;     // Moderate range for backing up
        conf.irRangeSide = 2.5;     // Good side awareness range
        
        conf.wheelSubstance.toRubber(40);  // Better wheels traction
        
        // Place the robot at a safe height above ground
        initialHeight = conf.size * 0.6;  // Height based on robot size
        vsg::dmat4 adjustedPose = vsg::translate(0.0, 0.0, initialHeight) * pose;
        
        // Create the robot with proper configuration
        FourWheeled* robot = new FourWheeled(odeHandle, vsgHandle, conf, name);
        // Enhanced visual appearance - distinctive blue color for sensor-enabled robot
        robot->setColor(Color(0.2, 0.4, 0.9));  // Bright blue to indicate advanced capabilities
        robot->place(adjustedPose);
        
        // Configure controller with appropriate parameters for sensor-rich robot
        SoxConf sc = Sox::getDefaultConf();
        sc.steps4Averaging = 2;  // More smoothing for better stability
        sc.modelInit = 1.0;      // Initial model weight for better adaptation
        AbstractController* controller = new Sox(sc);
        controller->setParam("epsC", 0.02);        // Lower learning rate for more stable control with sensors
        controller->setParam("epsA", 0.008);       // Reduced model learning rate for sensor integration
        controller->setParam("discountS", 0.98);   // Higher discount for sensor state learning
        controller->setParam("discountA", 0.95);   // Action discount for motor learning
        controller->setParam("s4avg", 2);          // More averaging for sensor noise reduction
        controller->setParam("s4delay", 1);        // Sensor delay compensation
        controller->setParam("harmony", 0.0);      // Deterministic behavior for obstacle navigation
        controller->setParam("epsC", 0.03);        // Final controller learning rate
        global.configs.push_back(controller);
        
        // Set up wiring and agent
        AbstractWiring* wiring = new One2OneWiring(new WhiteNormalNoise());
        OdeAgent* agent = new OdeAgent(global, PlotOption(NoPlot));
        agent->init(controller, robot, wiring);
        global.agents.push_back(agent);
        global.configs.push_back(agent);
        return agent;
    }
    
    // For regular Nimm2 robots, adjust height based on configuration
    initialHeight = nimm2conf.size * 0.6;
    vsg::dmat4 adjustedPose = vsg::translate(0.0, 0.0, initialHeight) * pose;
    
    OdeRobot* robot = new Nimm2(odeHandle, vsgHandle, nimm2conf, name);
    robot->setColor(Color(.1,.1,.8));
    robot->place(adjustedPose);
    
    SoxConf sc = Sox::getDefaultConf();
    sc.steps4Averaging = 1;
    AbstractController* controller = new Sox(sc);
    controller->setParam("epsC", 0.03);
    controller->setParam("epsA", 0.01);
    controller->setParam("discountS", 0.95);
    controller->setParam("discountA", 0.95);
    controller->setParam("s4avg", 1);
    controller->setParam("s4delay", 1);
    controller->setParam("harmony", 0.0);
    global.configs.push_back(controller);
    
    AbstractWiring* wiring = new One2OneWiring(new WhiteNormalNoise());
    OdeAgent* agent = new OdeAgent(global, PlotOption(NoPlot));
    agent->init(controller, robot, wiring);
    global.agents.push_back(agent);
    global.configs.push_back(agent);
    return agent;
}

void configureRobotPhysics(OdeHandle& odeHandle) {
    // Earth gravity
    dWorldSetGravity(odeHandle.world, 0, 0, -9.81);
    
    // Enhanced physics parameters for sensor-equipped robots
    // Error reduction parameter (how aggressively to correct joint errors)
    dWorldSetERP(odeHandle.world, 0.25);  // Slightly higher for better joint stability with sensors
    
    // Constraint force mixing (softness of constraints)
    dWorldSetCFM(odeHandle.world, 1e-6); // Harder constraints for sensor accuracy
    
    // Maximum correcting velocity for contacts
    dWorldSetContactMaxCorrectingVel(odeHandle.world, 80.0); // Optimized for sensor response
    
    // Depth of contact surface layer
    dWorldSetContactSurfaceLayer(odeHandle.world, 0.0008); // Slightly reduced for better precision
    
    // Solver iterations - more iterations = more accurate but slower
    dWorldSetQuickStepNumIterations(odeHandle.world, 40); // Balanced for sensor-motor coordination
    
    // Global damping for stability with sensor feedback
    dWorldSetLinearDamping(odeHandle.world, 0.004);  // Slightly reduced for more responsive movement
    dWorldSetAngularDamping(odeHandle.world, 0.006); // Slightly higher for rotational stability
    dWorldSetAutoDisableFlag(odeHandle.world, 0);    // Keep active for continuous sensor operation
}

void createGroundPlane(OdeHandle& odeHandle) {
    // Ground plane geometry for collision detection in ODE
    dGeomID ground = dCreatePlane(odeHandle.space, 0, 0, 1, -plane_z);
    dGeomSetCategoryBits(ground, 1);
    dGeomSetCollideBits(ground, 0xffffffff);
}

// void nearCallback(void* data, dGeomID o1, dGeomID o2) {
//     dBodyID b1 = dGeomGetBody(o1);
//     dBodyID b2 = dGeomGetBody(o2);
//     if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;
    
//     dContact contact;
//     // contact.surface.mode = dContactBounce | dContactSoftCFM;
//     // contact.surface.mu = 0.5;
//     // contact.surface.mu2 = 0.5;
//     // contact.surface.bounce = 0.1;
//     // contact.surface.bounce_vel = 0.1;
//     // contact.surface.soft_cfm = 0.01;
//     contact.surface.mode = dContactSoftCFM; // no bounce
//     contact.surface.bounce = 0.0;
//     contact.surface.bounce_vel = 0.0;
//     contact.surface.mu = 1.0;
//     contact.surface.mu2 = 1.0;


    
//     if (int numc = dCollide(o1, o2, 1, &contact.geom, sizeof(dContact))) {
//         dJointID c = dJointCreateContact(((OdeHandle*)data)->world, ((OdeHandle*)data)->jointGroup, &contact);
//         dJointAttach(c, b1, b2);
//     }
// }

int main(int argc, char** argv)
{
    try
    {
        // set up defaults and read command line arguments to override them
        vsg::CommandLine arguments(&argc, argv);

        // if we want to redirect std::cout and std::cerr to the vsg::Logger call vsg::Logger::redirect_stdout()
        if (arguments.read({"--redirect-std", "-r"})) vsg::Logger::instance()->redirect_std();

        // set up vsg::Options to pass in filepaths, ReaderWriters and other IO related options to use when reading and writing files.
        auto options = vsg::Options::create();
        options->sharedObjects = vsg::SharedObjects::create();
        options->fileCache = vsg::getEnv("VSG_FILE_CACHE");
        options->paths = vsg::getEnvPaths("VSG_FILE_PATH");

        arguments.read(options);
        auto windowTraits = vsg::WindowTraits::create();
        windowTraits->windowTitle = "vsgviewer";
        windowTraits->debugLayer = arguments.read({"--debug", "-d"});
        windowTraits->apiDumpLayer = arguments.read({"--api", "-a"});
        windowTraits->synchronizationLayer = arguments.read("--sync");
        bool reportAverageFrameRate = arguments.read("--fps");
        if (arguments.read("--double-buffer")) windowTraits->swapchainPreferences.imageCount = 2;
        if (arguments.read("--triple-buffer")) windowTraits->swapchainPreferences.imageCount = 3; // default
        if (arguments.read("--IMMEDIATE")) { windowTraits->swapchainPreferences.presentMode = VK_PRESENT_MODE_IMMEDIATE_KHR; }
        if (arguments.read("--FIFO")) windowTraits->swapchainPreferences.presentMode = VK_PRESENT_MODE_FIFO_KHR;
        if (arguments.read("--FIFO_RELAXED")) windowTraits->swapchainPreferences.presentMode = VK_PRESENT_MODE_FIFO_RELAXED_KHR;
        if (arguments.read("--MAILBOX")) windowTraits->swapchainPreferences.presentMode = VK_PRESENT_MODE_MAILBOX_KHR;
        if (arguments.read({"-t", "--test"}))
        {
            windowTraits->swapchainPreferences.presentMode = VK_PRESENT_MODE_IMMEDIATE_KHR;
            windowTraits->fullscreen = true;
            reportAverageFrameRate = true;
        }
        if (arguments.read({"--st", "--small-test"}))
        {
            windowTraits->swapchainPreferences.presentMode = VK_PRESENT_MODE_IMMEDIATE_KHR;
            windowTraits->width = 192, windowTraits->height = 108;
            windowTraits->decoration = false;
            reportAverageFrameRate = true;
        }

        bool multiThreading = arguments.read("--mt");
        if (arguments.read({"--fullscreen", "--fs"})) windowTraits->fullscreen = true;
        if (arguments.read({"--window", "-w"}, windowTraits->width, windowTraits->height)) { windowTraits->fullscreen = false; }
        if (arguments.read({"--no-frame", "--nf"})) windowTraits->decoration = false;
        if (arguments.read("--or")) windowTraits->overrideRedirect = true;
        auto maxTime = arguments.value(std::numeric_limits<double>::max(), "--max-time");

        if (arguments.read("--d32")) windowTraits->depthFormat = VK_FORMAT_D32_SFLOAT;
        if (arguments.read("--sRGB")) windowTraits->swapchainPreferences.surfaceFormat = {VK_FORMAT_B8G8R8A8_SRGB, VK_COLOR_SPACE_SRGB_NONLINEAR_KHR};
        if (arguments.read("--RGB")) windowTraits->swapchainPreferences.surfaceFormat = {VK_FORMAT_B8G8R8A8_UNORM, VK_COLOR_SPACE_SRGB_NONLINEAR_KHR};

        arguments.read("--screen", windowTraits->screenNum);
        arguments.read("--display", windowTraits->display);
        arguments.read("--samples", windowTraits->samples);
        if (int log_level = 0; arguments.read("--log-level", log_level)) vsg::Logger::instance()->level = vsg::Logger::Level(log_level);
        auto numFrames = arguments.value(-1, "-f");
        auto pathFilename = arguments.value<vsg::Path>("", "-p");
        auto loadLevels = arguments.value(0, "--load-levels");
        auto maxPagedLOD = arguments.value(0, "--maxPagedLOD");
        auto horizonMountainHeight = arguments.value(0.0, "--hmh");
        auto nearFarRatio = arguments.value<double>(0.001, "--nfr");
        if (arguments.read("--rgb")) options->mapRGBtoRGBAHint = false;

        bool depthClamp = arguments.read({"--dc", "--depthClamp"});
        if (depthClamp)
        {
            std::cout << "Enabled depth clamp." << std::endl;
            auto deviceFeatures = windowTraits->deviceFeatures = vsg::DeviceFeatures::create();
            deviceFeatures->get().samplerAnisotropy = VK_TRUE;
            deviceFeatures->get().depthClamp = VK_TRUE;
        }

        vsg::ref_ptr<vsg::ResourceHints> resourceHints;
        if (auto resourceHintsFilename = arguments.value<vsg::Path>("", "--rh"))
        {
            resourceHints = vsg::read_cast<vsg::ResourceHints>(resourceHintsFilename, options);
        }

        if (auto outputResourceHintsFilename = arguments.value<vsg::Path>("", "--orh"))
        {
            if (!resourceHints) resourceHints = vsg::ResourceHints::create();
            vsg::write(resourceHints, outputResourceHintsFilename, options);
            return 0;
        }

        if (int log_level = 0; arguments.read("--log-level", log_level)) vsg::Logger::instance()->level = vsg::Logger::Level(log_level);
        auto logFilename = arguments.value<vsg::Path>("", "--log");

        if (arguments.errors()) return arguments.writeErrorMessages(std::cerr);

        // Initialize ODE and VSG
        /**************** ODE-Section   ***********************/
        OdeHandle odeHandle;
        double simulationTime = 0.0;
        odeHandle.init(&simulationTime);
        global.odeConfig.setOdeHandle(odeHandle);
        configureRobotPhysics(odeHandle);
        // add ode config to config list
        global.configs.push_back(&(global.odeConfig));
        global.globalconfigurables.push_back(&(global.odeConfig));

        /**************** VulkanSceneGraph-Section   ***********************/
        VsgHandle vsgHandle;
        vsgHandle.init();

        // // Create ground plane
        // auto ground = new Plane();
        // ground->init(odeHandle, 0.0, vsgHandle, Primitive::Geom | Primitive::Draw);
        // ground->setPose(vsg::translate(0.0, 0.0, plane_z));
        // createGroundPlane(odeHandle);
        auto plane = makePhysicsScene(odeHandle, vsgHandle);
        vsgHandle.parent=vsgHandle.scene->scene;

        // Add boundary walls to prevent robot from falling off the edge
        // Parameters: odeHandle, vsgHandle, arenaSize (20.0), wallHeight (3.0)
        std::vector<Box*> boundaryWalls = addBoundaryWalls(odeHandle, vsgHandle, 5.0, 2.0);

        // Add some obstacles for the IR sensors to detect
        std::vector<Box*> obstacles;
        
        // Create a few obstacles for sensor testing
        for (int i = 0; i < 3; i++) {
            Box* obstacle = new Box(0.3, 0.3, 0.6);  // Small vertical obstacles
            obstacle->init(odeHandle, 0.0, vsgHandle.changeColor(0.8, 0.2, 0.2), Primitive::Geom | Primitive::Draw);
            
            // Position obstacles at different locations
            double x = (i - 1) * 1.5;  // Spread obstacles along X axis
            double y = 1.5 + i * 0.5;  // Offset in Y direction
            obstacle->setPose(vsg::translate(x, y, 0.3));  // Place on ground
            
            // Set obstacle material properties
            Substance obstacleSubstance;
            obstacleSubstance.toPlastic(30);  // Hard plastic obstacles
            obstacle->setSubstance(obstacleSubstance);
            
            obstacles.push_back(obstacle);
        }
        
        std::cout << "Created " << obstacles.size() << " test obstacles for IR sensor testing" << std::endl;

        // GlobalData global;
        global.vsgHandle = vsgHandle;
        
        // Create robot with slight random orientation for more interesting behavior
        double randomAngle = (rand() % 100) / 100.0 * M_PI / 3;  // Random angle up to 60 degrees
        vsg::dmat4 randomPose = vsg::rotate(randomAngle, vsg::dvec3(0.0, 0.0, 1.0)) * vsg::translate(0.0, 0.0, 0.0);
        
        auto agent = createVehicle(odeHandle, vsgHandle, global, randomPose, 4 /*FourWheeled*/);
        
        std::cout << "Created sensor-equipped robot with random starting orientation: " 
                  << (randomAngle * 180.0 / M_PI) << " degrees" << std::endl;
        // Enhanced tracking configuration for sensor-equipped robot
        TrackRobotConf conf;
        conf.trackPos              = true;       // Track position for navigation analysis
        conf.trackSpeed            = true;       // Track speed for performance evaluation
        conf.trackOrientation      = true;       // Track orientation for turning behavior
        conf.displayTrace          = true;       // Visual trace for path analysis
        conf.displayTraceDur       = 120;        // Longer duration for complex behaviors
        conf.displayTraceThickness = 2.5;        // Thicker line for better visibility
        conf.interval              = 1;          // Track every step for detailed analysis
        conf.writeFile             = true;       // Write comprehensive data to file
        
        // Enhanced file naming for sensor-equipped robot
        std::string robotName = agent->getRobot()->getName();
        char dirName[256];
        sprintf(dirName, "%s_sensor_equipped_track", robotName.c_str());
        createNewDir("./", dirName);
        
        // Use the full path for the tracking file
        char fullPath[512];
        sprintf(fullPath, "./%s/sensor_robot_track", dirName);
        conf.scene = std::string(fullPath);
        
        std::cout << "Creating enhanced tracking for sensor-equipped robot in: " << dirName << std::endl;
        std::cout << "Sensor robot tracking scene path: " << conf.scene << std::endl;
        
        agent->setTrackOptions(conf);

        // Give the robot an initial forward push to start exploring
        if (agent && agent->getRobot()) {
            // The controller will take over, but this gives it an initial direction
            std::cout << "Robot initialized with IR sensors enabled for obstacle detection" << std::endl;
            std::cout << "Expected behaviors: navigation, obstacle avoidance, exploration" << std::endl;
        }


        // Create robot
        // lpzrobots::HumanoidRobot humanoid;
        // humanoid.buildRobot(odeHandle, vsgHandle, true);

        // Nimm2Conf nimm2conf = Nimm2::getDefaultConf();
        // nimm2conf.sphereWheels = false; // normal wheels
        // // nimm2conf.size = 1;
        // // nimm2conf.force = 5;
        // // nimm2conf.speed=20;
        // // //      nimm2conf.speed=15;
        // // nimm2conf.cigarMode=true;
        // // nimm2conf.cigarLength= 5.0;
        // // nimm2conf.singleMotor=false;
        // // nimm2conf.boxMode=true;
        // nimm2conf.boxWidth=10;
        // // //      nimm2conf.visForce =true;
        // // nimm2conf.bumper=true;
        // OdeRobot* robot = new Nimm2(odeHandle, vsgHandle, nimm2conf, "nimm2");
        // robot->placeIntern(vsg::rotate(M_PI * 2.0, vsg::dvec3(0.0,1.0,0.0)) /** vsg::translate(-2.0, 0.0, 0.0)*/);

        // Box* sphere = new Box(1.4, 0.8, 0.4);
        // sphere->init(odeHandle, 1.0, vsgHandle, Primitive::Body | Primitive::Geom | Primitive::Draw);
        // sphere->setPose(vsg::translate(0.0, 0.0, 2.0) * vsg::rotate(M_PI/2.0,1.0,0.0,0.0));
        // std::cout << "Sphere is positioned at: " << std::endl;
        // sphere->getPosition().print();

        // std::cout<< robot->getName() << std::endl;
        // robot->getPosition().print();

        // robot->setColor(Color(.1,.1,.8));
        // robot->place(pose);
        // std::cout << "Robot created" << std::endl;

        // Wait a few simulation steps for physics to settle
        // for (int i = 0; i < 10; i++) {
        //     simulateStep(odeHandle, 0.01);
        //     robot->update();
        // }


        vsg::ref_ptr<vsg::Node> vsg_scene;
        if (vsgHandle.parent->children.size() == 1)
            vsg_scene = vsgHandle.parent->children[0];
        else
            vsg_scene = vsgHandle.parent;

        // create the viewer and assign window(s) to it
        auto viewer = vsg::Viewer::create();
        auto window = vsg::Window::create(windowTraits);
        if (!window)
        {
            std::cout << "Could not create window." << std::endl;
            return 1;
        }

        viewer->addWindow(window);

        // compute the bounds of the scene graph to help position camera
        vsg::ComputeBounds computeBounds;
        vsg_scene->accept(computeBounds);
        vsg::dvec3 centre = (computeBounds.bounds.min + computeBounds.bounds.max) * 0.5;
        double radius = vsg::length(computeBounds.bounds.max - computeBounds.bounds.min) * 0.6;

        // set up the camera
        auto lookAt = vsg::LookAt::create(centre + vsg::dvec3(0.0, -radius * 3.5, 0.0), centre, vsg::dvec3(0.0, 0.0, 1.0));

        vsg::ref_ptr<vsg::ProjectionMatrix> perspective;
        auto ellipsoidModel = vsg_scene->getRefObject<vsg::EllipsoidModel>("EllipsoidModel");
        if (ellipsoidModel)
        {
            perspective = vsg::EllipsoidPerspective::create(lookAt, ellipsoidModel, 30.0, static_cast<double>(window->extent2D().width) / static_cast<double>(window->extent2D().height), nearFarRatio, horizonMountainHeight);
        }
        else
        {
            perspective = vsg::Perspective::create(30.0, static_cast<double>(window->extent2D().width) / static_cast<double>(window->extent2D().height), nearFarRatio * radius, radius * 4.5);
        }

        auto camera = vsg::Camera::create(perspective, lookAt, vsg::ViewportState::create(window->extent2D()));

        // add close handler to respond to the close window button and pressing escape
        viewer->addEventHandler(vsg::CloseHandler::create(viewer));

        auto cameraAnimation = vsg::CameraAnimation::create(camera, pathFilename, options);
        viewer->addEventHandler(cameraAnimation);
        viewer->addEventHandler(vsg::Trackball::create(camera, ellipsoidModel));

        auto commandGraph = vsg::createCommandGraphForView(window, camera, vsg_scene);
        viewer->assignRecordAndSubmitTaskAndPresentation({commandGraph});

        viewer->compile(resourceHints);

        viewer->start_point() = vsg::clock::now();
        
        // Frame counter for monitoring
        int frameCount = 0;

        // rendering main loop
        while (viewer->advanceToNextFrame() && (numFrames < 0 || (numFrames--) > 0) && (viewer->getFrameStamp()->simulationTime < maxTime))
        {
            viewer->handleEvents();

            try {
                frameCount++;
                
                //********************Simulation start********************************
                // Display sensor info periodically
                if (!global.agents.empty()) {
                    displaySensorInfo(global.agents[0], frameCount);
                }
                
                // First update all agents and let them control their robots
                FOREACH(OdeAgentList, global.agents, i) {
                    (*i)->beforeStep(global);
                    (*i)->step(global.odeConfig.noise, global.time);
                }
                
                // Apply all motor commands to the robots
                FOREACH(OdeAgentList, global.agents, i) {
                    (*i)->setMotorsGetSensors();
                    (*i)->getRobot()->doInternalStuff(global);
                }
                
                // Initialize any temporary objects that were added
                global.initializeTmpObjects(odeHandle, vsgHandle);
                
                // Perform physics simulation step
                odeStep(odeHandle, 0.01);
                
                // Update global time
                global.time += 0.01;
                
                // Update all primitives to sync VSG with ODE positions
                FOREACH(OdeAgentList, global.agents, i) {
                    (*i)->getRobot()->update();
                }
                
                if (plane) plane->update();
                
                // Update boundary walls if any exist
                for (Box* wall : boundaryWalls) {
                    if (wall) wall->update();
                }
                
                // Update obstacles
                for (Box* obstacle : obstacles) {
                    if (obstacle) obstacle->update();
                }
                
                // Update and render the scene
                viewer->update();
                viewer->recordAndSubmit();
                viewer->present();

            } catch (const std::exception& e) {
                std::cerr << "Error in simulation loop: " << e.what() << std::endl;
                std::cerr << "Frame: " << frameCount << ", Time: " << global.time << std::endl;
                
                // Try to recover by skipping this frame
                frameCount++;
                if (frameCount % 1000 == 0) {
                    std::cout << "Simulation continuing... Frame " << frameCount 
                              << ", Time: " << global.time << "s" << std::endl;
                }
                continue;
            } catch (...) {
                std::cerr << "Unknown error in simulation loop at frame " << frameCount << std::endl;
                break;
            }
        }

        // // Destroy the robot first, ensuring ODE is still valid:
        // if (ground) {
        //     delete ground;
        // }

        // Clean up the boundary walls
        for (Box* wall : boundaryWalls) {
            if (wall) delete wall;
        }
        boundaryWalls.clear();
        
        // Clean up obstacles
        for (Box* obstacle : obstacles) {
            if (obstacle) delete obstacle;
        }
        obstacles.clear();

        // Now that the robot and its joints are safely destroyed, 
        // you can close ODE and VSG:
        odeHandle.close();
        vsgHandle.close();


        if (reportAverageFrameRate)
        {
            auto fs = viewer->getFrameStamp();
            double fps = static_cast<double>(fs->frameCount) / std::chrono::duration<double, std::chrono::seconds::period>(vsg::clock::now() - viewer->start_point()).count();
            std::cout << "Average frame rate = " << fps << " fps" << std::endl;
        }
    }
    catch (const vsg::Exception& ve)
    {
        for (int i = 0; i < argc; ++i) std::cerr << argv[i] << " ";
        std::cerr << "\n[Exception] - " << ve.message << " result = " << ve.result << std::endl;
        return 1;
    }

    // clean up done automatically thanks to ref_ptr<>
    return 0;
}