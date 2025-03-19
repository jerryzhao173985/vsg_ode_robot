// primitive.cpp
#include "primitive.h"
#include "odehandle.h"
#include "vsghandle.h"
#include <iostream>
#include <cassert>

#include <vsg/all.h>
#include <vsgXchange/all.h>
#include <iostream>

namespace lpzrobots {

// Find the color node and change its color
struct ChangeColor : public vsg::Visitor
{
    // function to set color to change to
    ChangeColor(const vsg::vec4& color) : color(color) {}
    
    void apply(vsg::Object& object) override
    {
        object.traverse(*this);
    }

    void apply(vsg::VertexIndexDraw& vid) override
    {
        for (auto& buffer : vid.arrays) {
            auto data = buffer->data;
            if (data->is_compatible(typeid(vsg::vec4Array))) {
                // Change the color to green
                (*(vsg::vec4*)(data->dataPointer(0))) = color;
            }
        }
    }
    vsg::vec4 color;
};

int globalNumVelocityViolations = 0;
bool Primitive::destroyGeom = true;

// Helper function for matrix conversion
vsg::dmat4 odeToVsgMatrix(const dReal* position, const dReal* rotation) {
    if (!position || !rotation) {
        std::cerr << "odeToVsgMatrix: Null position or rotation, returning identity matrix" << std::endl;
        return vsg::dmat4(1.0); // Identity matrix
    }

    try {
        return vsg::dmat4(
            rotation[0], rotation[1], rotation[2], 0.0,
            rotation[4], rotation[5], rotation[6], 0.0,
            rotation[8], rotation[9], rotation[10], 0.0,
            position[0], position[1], position[2], 1.0
        );
    } catch (const std::exception& e) {
        std::cerr << "odeToVsgMatrix: Exception during matrix creation: " << e.what() << std::endl;
        return vsg::dmat4(1.0); // Identity matrix as fallback
    }
}


// Helper functions
Pose vsgPose(dGeomID geom) {
    const dReal* pos = dGeomGetPosition(geom);
    const dReal* rot = dGeomGetRotation(geom);
    return Pose(odeToVsgMatrix(pos, rot));
}

Pose vsgPose(dBodyID body) {
    const dReal* pos = dBodyGetPosition(body);
    const dReal* rot = dBodyGetRotation(body);
    return Pose(odeToVsgMatrix(pos, rot));
}

Pose vsgPose(const double* position, const double* rotation) {
    return Pose(odeToVsgMatrix(position, rotation));
}

// Helper function to convert VSG matrix rotation to ODE format for Pose input
void odeRotation(const Pose& pose, dMatrix3& odematrix) {
    //     // ODE matrix is stored in row-major order with indices:
    //     // [0 1 2 3]    0  1  2  (3 unused)
    //     // [4 5 6 7] =  4  5  6  (7 unused)
    //     // [8 9 10 11]  8  9  10 (11 unused)
    // transposing from VSG column-major to ODE row-major
    vsg::dmat4 mat = pose.getMatrix();
    odematrix[0] = mat[0][0]; odematrix[1] = mat[0][1]; odematrix[2] = mat[0][2];
    odematrix[4] = mat[1][0]; odematrix[5] = mat[1][1]; odematrix[6] = mat[1][2];
    odematrix[8] = mat[2][0]; odematrix[9] = mat[2][1]; odematrix[10] = mat[2][2];
    // Set unused elements to 0
    odematrix[3] = odematrix[7] = odematrix[11] = 0.0;
}

// Primitive base class implementation
Primitive::Primitive() 
    : geom(nullptr), body(nullptr), mode(0), 
      substanceManuallySet(false), numVelocityViolations(0),
      vsgPrimitive(nullptr), transformNode(nullptr), options(nullptr),
      color(1.0, 1.0, 1.0, 1.0) {
    // if options not defined then define them here
    if(!options) {
        options = vsg::Options::create();
        options->sharedObjects = vsg::SharedObjects::create();
        options->fileCache = vsg::getEnv("VSG_FILE_CACHE");
        options->paths = vsg::getEnvPaths("VSG_FILE_PATH");
    }
}

Primitive::~Primitive() {
    if (destroyGeom && geom) {
        dGeomDestroy(geom);
    }
    if (body && ((mode & _Transform) == 0)) {
        dBodyDestroy(body);
    }
    transformNode = nullptr;
}

void Primitive::attachGeomAndSetColliderFlags() {
    if (!geom) return;
    
    if (mode & Body) {
        dGeomSetBody(geom, body);
        dGeomSetCategoryBits(geom, Dyn);
        dGeomSetCollideBits(geom, ~0x0);
    } else {
        dGeomSetCategoryBits(geom, Stat);
        dGeomSetCollideBits(geom, ~Stat);
    }
    if (mode & _Child) {
        dGeomSetCategoryBits(geom, Dyn);
        dGeomSetCollideBits(geom, ~0x0);
    }
    dGeomSetData(geom, (void*)this);
}

void Primitive::setColor(const Color& color){
    ChangeColor fc(color);
    vsgPrimitive->accept(fc);
    this->color = Color(color);
}

void Primitive::setPosition(const Pos& pos) {
    if (body) {
        dBodySetPosition(body, pos.x(), pos.y(), pos.z());
    } 
    else if (geom) {
        dGeomSetPosition(geom, pos.x(), pos.y(), pos.z());
    }
    update();
}

void Primitive::setPose(const Pose& pose) {
    const vsg::dmat4& mat = pose.getMatrix();
    
    if (body) {
        dBodySetPosition(body, mat[3][0], mat[3][1], mat[3][2]);
        dMatrix3 R;
        odeRotation(pose, R);
        dBodySetRotation(body, R);
    } else if (geom) {
        dGeomSetPosition(geom, mat[3][0], mat[3][1], mat[3][2]);
        dMatrix3 R;
        odeRotation(pose, R);
        dGeomSetRotation(geom, R);
    }

    if (transformNode) {
        transformNode->matrix = mat;
    }

    update();
}

// set the transform node matrix
void Primitive::setMatrix(const vsg::dmat4& matrix) {
    if (transformNode) {
        transformNode->matrix = matrix;
    }
}

// In Primitive::update()
void Primitive::update() {
    if (mode & Draw && transformNode) {
        try {
            if (!geom && !body) {
                return; // Nothing to update if no physics objects exist
            }
            
            const vsg::dmat4& mat = getPose().getMatrix();
            transformNode->matrix = mat;
        } catch (const std::exception& e) {
            std::cerr << "Error in Primitive::update(): " << e.what() << std::endl;
        }
    }
}


Pos Primitive::getPosition() const {
    if (geom) {
        const dReal* pos = dGeomGetPosition(geom);
        return Pos(pos[0], pos[1], pos[2]);
    } else if (body) {
        const dReal* pos = dBodyGetPosition(body);
        return Pos(pos[0], pos[1], pos[2]);
    }
    return Pos(0, 0, 0);
}

Pose Primitive::getPose() const {
    if (geom) {
        return vsgPose(geom);
    } else if (body) {
        return vsgPose(body);
    }
    // Return identity pose when neither geom nor body exists
    return Pose(vsg::dmat4(1.0));
}

Pos Primitive::getVel() const {
    if (body) {
        const dReal* vel = dBodyGetLinearVel(body);
        return Pos(vel[0], vel[1], vel[2]);
    }
    return Pos(0, 0, 0);
}

Pos Primitive::getAngularVel() const {
    if (body) {
        const dReal* vel = dBodyGetAngularVel(body);
        return Pos(vel[0], vel[1], vel[2]);
    }
    return Pos(0, 0, 0);
}

bool Primitive::applyForce(vsg::dvec3 force) {
    if (body) {
        dBodyAddForce(body, force.x, force.y, force.z);
        return true;
    }
    return false;
}

bool Primitive::applyForce(double x, double y, double z) {
    if (body) {
        dBodyAddForce(body, x, y, z);
        return true;
    }
    return false;
}

bool Primitive::applyTorque(vsg::dvec3 torque) {
    if (body) {
        dBodyAddTorque(body, torque.x, torque.y, torque.z);
        return true;
    }
    return false;
}

bool Primitive::applyTorque(double x, double y, double z) {
    if (body) {
        dBodyAddTorque(body, x, y, z);
        return true;
    }
    return false;
}

void Primitive::setMass(double mass, double cgx, double cgy, double cgz,
                       double I11, double I22, double I33,
                       double I12, double I13, double I23) {
    if (!body) return;
    
    dMass m;
    dMassSetParameters(&m, mass, cgx, cgy, cgz, I11, I22, I33, I12, I13, I23);
    dBodySetMass(body, &m);
}

bool Primitive::limitLinearVel(double maxVel) {
    if (!body) return false;
    
    const dReal* vel = dBodyGetLinearVel(body);
    double vellen = vel[0]*vel[0] + vel[1]*vel[1] + vel[2]*vel[2];
    
    if (vellen > maxVel*maxVel) {
        double scaling = sqrt(vellen) / maxVel;
        dBodySetLinearVel(body, 
            vel[0]/scaling, vel[1]/scaling, vel[2]/scaling);
        numVelocityViolations++;
        globalNumVelocityViolations++;
        return true;
    }
    return false;
}

bool Primitive::limitAngularVel(double maxVel) {
    if (!body) return false;
    
    const dReal* vel = dBodyGetAngularVel(body);
    double vellen = vel[0]*vel[0] + vel[1]*vel[1] + vel[2]*vel[2];
    
    if (vellen > maxVel*maxVel) {
        double scaling = sqrt(vellen) / maxVel;
        dBodySetAngularVel(body,
            vel[0]/scaling, vel[1]/scaling, vel[2]/scaling);
        numVelocityViolations++;
        globalNumVelocityViolations++;
        return true;
    }
    return false;
}

void Primitive::decelerate(double factorLin, double factorAng) {
    if (!body) return;
    
    if (factorLin != 0) {
        Pos vel = getVel();
        applyForce(-vel.x()*factorLin, -vel.y()*factorLin, -vel.z()*factorLin);
    }
    if (factorAng != 0) {
        Pos avel = getAngularVel();
        applyTorque(-avel.x()*factorAng, -avel.y()*factorAng, -avel.z()*factorAng);
    }
}


vsg::dvec3 Primitive::toLocal(const vsg::dvec3& pos) const {
    Pose invPose = getPose().inverse();
    vsg::dvec4 temp(pos.x, pos.y, pos.z, 1.0);
    temp = invPose.getMatrix() * temp;
    return vsg::dvec3(temp.x, temp.y, temp.z);
}

vsg::dvec4 Primitive::toLocal(const vsg::dvec4& axis) const {
    return getPose().inverse().getMatrix() * axis;
}


vsg::dvec3 Primitive::toGlobal(const vsg::dvec3& pos) const {
    vsg::dvec4 temp(pos.x, pos.y, pos.z, 1.0);
    temp = getPose().getMatrix() * temp;
    return vsg::dvec3(temp.x, temp.y, temp.z);
}

vsg::dvec4 Primitive::toGlobal(const vsg::dvec4& axis) const {
    return getPose().getMatrix() * axis;
}

void Primitive::setSubstance(const Substance& substance) {
    this->substance = substance;
    substanceManuallySet = true;
}


bool Primitive::store(FILE* f) const {
    Pose pose = getPose();
    Pos vel = getVel();
    Pos avel = getAngularVel();
    
    // Store matrix data
    if (fwrite(static_cast<vsg::dmat4>(pose).data(), sizeof(double), 16, f) != 16) return false;
    
    // Store velocity data
    double velData[3] = {vel.x(), vel.y(), vel.z()};
    double avelData[3] = {avel.x(), avel.y(), avel.z()};
    
    if (fwrite(velData, sizeof(double), 3, f) != 3) return false;
    if (fwrite(avelData, sizeof(double), 3, f) != 3) return false;
    
    return true;
}

bool Primitive::restore(FILE* f) {
    vsg::dmat4 mat;
    double velData[3], avelData[3];
    
    if (fread(mat.data(), sizeof(double), 16, f) != 16) return false;
    if (fread(velData, sizeof(double), 3, f) != 3) return false;
    if (fread(avelData, sizeof(double), 3, f) != 3) return false;
    
    setPose(Pose(mat));
    if (body) {
        dBodySetLinearVel(body, velData[0], velData[1], velData[2]);
        dBodySetAngularVel(body, avelData[0], avelData[1], avelData[2]);
    }
    
    return true;
}

// Implementation of specific primitives continues in the next section...
// Continuation of primitive.cpp - Specific primitive implementations

// Plane implementation
Plane::Plane() : Primitive() {
}

Plane::~Plane() {
}

void Plane::init(const OdeHandle& odeHandle, double mass,
                 const VsgHandle& vsgHandle, char mode) {
    assert(mode & Geom);
    vsg::Builder builder;
    builder.options = options; // use VSG options
    vsg::GeometryInfo geom;
    geom.position = {0.0f, 0.0f, 0.0f};  // Centered at origin
    geom.dx = {5.0f, 0.0f, 0.0f};        // 5 units along X-axis
    geom.dy = {0.0f, 5.0f, 0.0f};        // 5 units along Y-axis
    geom.dz.set(0.0f, 0.0f, 0.01f);
    // geom.dy.set(0.0f, 0.0f, 1.0f);
    // geom.dz.set(0.0f, -1.0f, 0.0f);
    geom.color = vsg::vec4(0.3f, 0.3f, 0.3f, 1.0f);  // Grey color for visibility
    vsg::StateInfo state;
    state.lighting = false;  // Disable lighting
    vsgPrimitive = builder.createQuad(geom, state);
    color = Color(geom.color);
    
    transformNode = vsg::MatrixTransform::create();
    transformNode->matrix = vsg::dmat4(1.0f);
    transformNode->addChild(vsgPrimitive);
    
    // Apply state if provided
    if (vsgHandle.cfg && vsgHandle.cfg->normalState) {
        transformNode->addChild(vsgHandle.cfg->normalState);
    }

    // add transformed primitive to the parent group in vsgHandle
    if (vsgHandle.parent) {
        vsgHandle.parent->addChild(transformNode);
    }
}


void Plane::update() {
    // std::cout << "Plane Update complete" << std::endl;
    // Planes are static, so no update needed
}

vsg::ref_ptr<vsg::MatrixTransform> Plane::getTransformNode() {
    return transformNode;
}

void Plane::setMass(double mass, bool density) {
    // Planes don't have mass
}

// Box Implementation
Box::Box(double lengthX, double lengthY, double lengthZ)
    : Primitive(), dimensions(lengthX, lengthY, lengthZ) {
}

Box::Box(const vsg::dvec3& dim)
    : Primitive(), dimensions(dim) {
}

Box::~Box() {
}


void Box::init(const OdeHandle& odeHandle, double mass, const VsgHandle& vsgHandle, char mode) {
    if (!(mode & (Body | Geom))) {
        std::cerr << "Box::init: Invalid mode, needs Body or Geom" << std::endl;
        return;
    }

    this->mode = mode;

    // Physics initialization remains the same
    if (mode & Body) {
        body = dBodyCreate(odeHandle.world);
        setMass(mass, mode & Density);
    }

    if (mode & Geom) {
        geom = dCreateBox(odeHandle.space, dimensions.x, dimensions.y, dimensions.z);
        attachGeomAndSetColliderFlags();
    }

    if (mode & Draw) {
        try {
            float dx = static_cast<float>(dimensions.x * 0.5f);
            float dy = static_cast<float>(dimensions.y * 0.5f);
            float dz = static_cast<float>(dimensions.z * 0.5f);
            
            vsg::Builder builder;
            builder.options = options; // use VSG options
            vsg::GeometryInfo geom;
            geom.position = {0.0f, 0.0f, 0.0f};  // Centered at origin
            geom.dx = {dx, 0.0f, 0.0f};        // 5 units along X-axis
            geom.dy = {0.0f, dy, 0.0f};        // 5 units along Y-axis
            geom.dz = {0.0f, 0.0f, dz};        // 5 units along Z-axis
            geom.color = vsg::vec4(1.0f, 0.0f, 0.0f, 1.0f);

            vsg::StateInfo state;
            state.lighting = false;  // Disable lighting
            vsgPrimitive = builder.createBox(geom, state);
            color = Color(geom.color);
            
            transformNode = vsg::MatrixTransform::create();
            transformNode->matrix = vsg::dmat4(1.0f);
            transformNode->addChild(vsgPrimitive);
            
            // Apply state if provided
            if (vsgHandle.cfg && vsgHandle.cfg->normalState) {
                transformNode->addChild(vsgHandle.cfg->normalState);
            }

            // add transformed primitive to the parent group in vsgHandle
            if (vsgHandle.parent) {
                vsgHandle.parent->addChild(transformNode);
            }

        } catch (const std::exception& e) {
            std::cerr << "Box::init: Exception during visual setup: " << e.what() << std::endl;
        }
    }
}


void Box::update() {
    if (!(mode & Draw) || !transformNode) return;

    try {
        transformNode->matrix = getPose().getMatrix();
    } catch (const std::exception& e) {
        std::cerr << "Box::update: " << e.what() << std::endl;
    }
}



vsg::ref_ptr<vsg::MatrixTransform> Box::getTransformNode() {
    return transformNode;
}

void Box::setMass(double mass, bool density) {
    if (!body) return;

    dMass m;
    if (density) {
        dMassSetBox(&m, mass, dimensions.x, dimensions.y, dimensions.z);
    } else {
        dMassSetBoxTotal(&m, mass, dimensions.x, dimensions.y, dimensions.z);
    }
    dBodySetMass(body, &m);
}

dReal Box::getMass() const {
    if (!body) return 0.0;
    
    dMass m;
    dBodyGetMass(body, &m);
    return m.mass;
}

dReal Box::getVolume() const {
    return dimensions.x * dimensions.y * dimensions.z;
}

dReal Box::getDensity() const {
    return getMass() / getVolume();
}


// Sphere implementation
Sphere::Sphere(double radius) 
    : Primitive(), radius(radius) {
}

Sphere::~Sphere() {
}



void Sphere::init(const OdeHandle& odeHandle, double mass,
                 const VsgHandle& vsgHandle, char mode) {
    assert(mode & (Body | Geom));
    
    if (!substanceManuallySet)
        substance = odeHandle.substance;
    this->mode = mode;
    
    if (mode & Body) {
        body = dBodyCreate(odeHandle.world);
        setMass(mass, mode & Density);
    }
    
    if (mode & Geom) {
        geom = dCreateSphere(odeHandle.space, radius);
        attachGeomAndSetColliderFlags();
    }
    
    if (mode & Draw) {
        vsg::Builder builder;
        builder.options = options; // use VSG options
        vsg::GeometryInfo geom;
        geom.position = {0.0f, 0.0f, 0.0f};      // Centered at origin
        geom.dx = {radius, 0.0f, 0.0f};        
        geom.dy = {0.0f, radius, 0.0f};        
        geom.dz = {0.0f, 0.0f, radius};        
        geom.color.set(0.1f,0.1f,0.8f,1.0f);     // Blue color by default sphere

        vsg::StateInfo state;
        state.lighting = false;  // Disable lighting
        vsgPrimitive = builder.createSphere(geom, state);
        color = Color(geom.color);
        
        transformNode = vsg::MatrixTransform::create();
        transformNode->matrix = vsg::dmat4(1.0f);
        transformNode->addChild(vsgPrimitive);
        
        // Apply state if provided
        if (vsgHandle.cfg && vsgHandle.cfg->normalState) {
            transformNode->addChild(vsgHandle.cfg->normalState);
        }

        // add transformed primitive to the parent group in vsgHandle
        if (vsgHandle.parent) {
            vsgHandle.parent->addChild(transformNode);
        }
    }
}

void Sphere::update() {
    if (mode & Draw && transformNode) {
        try {
            transformNode->matrix = getPose().getMatrix();
        } catch (const std::exception& e) {
            std::cerr << "Sphere::update: " << e.what() << std::endl;
        }
    }
}

vsg::ref_ptr<vsg::MatrixTransform> Sphere::getTransformNode() {
    return transformNode;
}

void Sphere::setMass(double mass, bool density) {
    if (!body) return;
    
    dMass m;
    if (density) {
        dMassSetSphere(&m, mass, radius);
    } else {
        dMassSetSphereTotal(&m, mass, radius);
    }
    dBodySetMass(body, &m);
}


// The rest of other classes are implemented following the same pattern
// primitive.cpp additions

// Capsule Implementation
Capsule::Capsule(double radius, double height)
    : Primitive(), radius(radius), height(height) {
}

Capsule::~Capsule() {
}

void Capsule::init(const OdeHandle& odeHandle, double mass,
                   const VsgHandle& vsgHandle, char mode) {
    assert(mode & (Body | Geom));
    
    if (!substanceManuallySet)
        substance = odeHandle.substance;
    this->mode = mode;
    
    if (mode & Body) {
        body = dBodyCreate(odeHandle.world);
        setMass(mass, mode & Density);
    }
    
    if (mode & Geom) {
        geom = dCreateCapsule(odeHandle.space, radius, height);
        attachGeomAndSetColliderFlags();
    }
    
    if (mode & Draw) {
        vsg::Builder builder;
        builder.options = options;
        vsg::GeometryInfo geom;
        geom.position = {0.0f, 0.0f, 0.0f};
        geom.dx = {radius, 0.0f, 0.0f};
        geom.dy = {0.0f, radius, 0.0f};
        geom.dz = {0.0f, 0.0f, height * 0.5f};
        geom.color = vsg::vec4(0.8f, 0.1f, 0.1f, 1.0f);

        vsg::StateInfo state;
        state.lighting = false;
        vsgPrimitive = builder.createCapsule(geom, state);
        color = Color(geom.color);
        
        transformNode = vsg::MatrixTransform::create();
        transformNode->matrix = vsg::dmat4(1.0f);
        transformNode->addChild(vsgPrimitive);
        
        if (vsgHandle.cfg && vsgHandle.cfg->normalState) {
            transformNode->addChild(vsgHandle.cfg->normalState);
        }

        if (vsgHandle.parent) {
            vsgHandle.parent->addChild(transformNode);
        }
    }
}

void Capsule::update() {
    if (mode & Draw && transformNode) {
        try {
            transformNode->matrix = getPose().getMatrix();
        } catch (const std::exception& e) {
            std::cerr << "Capsule::update: " << e.what() << std::endl;
        }
    }
}

vsg::ref_ptr<vsg::MatrixTransform> Capsule::getTransformNode() {
    return transformNode;
}

void Capsule::setMass(double mass, bool density) {
    if (!body) return;
    
    dMass m;
    if (density) {
        dMassSetCapsule(&m, mass, 3, radius, height); // 3 = z-axis alignment
    } else {
        dMassSetCapsuleTotal(&m, mass, 3, radius, height);
    }
    dBodySetMass(body, &m);
}

// Cone Implementation
Cone::Cone(double radius, double height)
    : Primitive(), radius(radius), height(height) {
}

Cone::~Cone() {
}

void Cone::init(const OdeHandle& odeHandle, double mass,
                const VsgHandle& vsgHandle, char mode) {
    assert(mode & (Body | Geom));
    
    if (!substanceManuallySet)
        substance = odeHandle.substance;
    this->mode = mode;
    
    if (mode & Body) {
        body = dBodyCreate(odeHandle.world);
        setMass(mass, mode & Density);
    }
    
    if (mode & Geom) {
        // ODE doesn't have a native cone shape, use cylinder as approximation
        geom = dCreateCylinder(odeHandle.space, radius, height);
        attachGeomAndSetColliderFlags();
    }
    
    if (mode & Draw) {
        vsg::Builder builder;
        builder.options = options;
        vsg::GeometryInfo geom;
        geom.position = {0.0f, 0.0f, 0.0f};
        geom.dx = {radius, 0.0f, 0.0f};
        geom.dy = {0.0f, radius, 0.0f};
        geom.dz = {0.0f, 0.0f, height};
        geom.color = vsg::vec4(0.1f, 0.8f, 0.1f, 1.0f);

        vsg::StateInfo state;
        state.lighting = false;
        vsgPrimitive = builder.createCone(geom, state);
        color = Color(geom.color);
        
        transformNode = vsg::MatrixTransform::create();
        transformNode->matrix = vsg::dmat4(1.0f);
        transformNode->addChild(vsgPrimitive);
        
        if (vsgHandle.cfg && vsgHandle.cfg->normalState) {
            transformNode->addChild(vsgHandle.cfg->normalState);
        }

        if (vsgHandle.parent) {
            vsgHandle.parent->addChild(transformNode);
        }
    }
}

void Cone::update() {
    if (mode & Draw && transformNode) {
        try {
            transformNode->matrix = getPose().getMatrix();
        } catch (const std::exception& e) {
            std::cerr << "Cone::update: " << e.what() << std::endl;
        }
    }
}

vsg::ref_ptr<vsg::MatrixTransform> Cone::getTransformNode() {
    return transformNode;
}

void Cone::setMass(double mass, bool density) {
    if (!body) return;
    
    dMass m;
    if (density) {
        // Approximate cone mass with cylinder
        dMassSetCylinder(&m, mass, 3, radius, height);
    } else {
        dMassSetCylinderTotal(&m, mass, 3, radius, height);
    }
    dBodySetMass(body, &m);
}

// Cylinder Implementation
Cylinder::Cylinder(double radius, double height)
    : Primitive(), radius(radius), height(height) {
}

Cylinder::~Cylinder() {
}

void Cylinder::init(const OdeHandle& odeHandle, double mass,
                    const VsgHandle& vsgHandle, char mode) {
    // TODO:Properly fix this for tracking line!!
    // assert(mode & (Body | Geom));
    
    if (!substanceManuallySet)
        substance = odeHandle.substance;
    this->mode = mode;
    
    if (mode & Body) {
        body = dBodyCreate(odeHandle.world);
        setMass(mass, mode & Density);
    }
    
    if (mode & Geom) {
        geom = dCreateCylinder(odeHandle.space, radius, height);
        attachGeomAndSetColliderFlags();
    }
    
    if (mode & Draw) {
        vsg::Builder builder;
        builder.options = options;
        vsg::GeometryInfo geom;
        geom.position = {0.0f, 0.0f, 0.0f};
        geom.dx = {radius, 0.0f, 0.0f};
        geom.dy = {0.0f, radius, 0.0f};
        geom.dz = {0.0f, 0.0f, height * 0.5f};
        geom.color = vsg::vec4(0.1f, 0.1f, 0.8f, 1.0f);

        vsg::StateInfo state;
        state.lighting = false;
        vsgPrimitive = builder.createCylinder(geom, state);
        color = Color(geom.color);
        
        transformNode = vsg::MatrixTransform::create();
        transformNode->matrix = vsg::dmat4(1.0f);
        transformNode->addChild(vsgPrimitive);
        
        if (vsgHandle.cfg && vsgHandle.cfg->normalState) {
            transformNode->addChild(vsgHandle.cfg->normalState);
        }

        if (vsgHandle.parent) {
            vsgHandle.parent->addChild(transformNode);
        }
    }
}

void Cylinder::update() {
    if (mode & Draw && transformNode) {
        try {
            transformNode->matrix = getPose().getMatrix();
        } catch (const std::exception& e) {
            std::cerr << "Cylinder::update: " << e.what() << std::endl;
        }
    }
}

vsg::ref_ptr<vsg::MatrixTransform> Cylinder::getTransformNode() {
    return transformNode;
}

void Cylinder::setMass(double mass, bool density) {
    if (!body) return;
    
    dMass m;
    if (density) {
        dMassSetCylinder(&m, mass, 3, radius, height);
    } else {
        dMassSetCylinderTotal(&m, mass, 3, radius, height);
    }
    dBodySetMass(body, &m);
}

// Disk Implementation
Disk::Disk(double radius)
    : Primitive(), radius(radius) {
}

Disk::~Disk() {
}

void Disk::init(const OdeHandle& odeHandle, double mass,
                const VsgHandle& vsgHandle, char mode) {
    assert(mode & (Body | Geom));
    
    if (!substanceManuallySet)
        substance = odeHandle.substance;
    this->mode = mode;
    
    if (mode & Body) {
        body = dBodyCreate(odeHandle.world);
        setMass(mass, mode & Density);
    }
    
    if (mode & Geom) {
        // ODE doesn't have a native disk shape, use thin cylinder
        geom = dCreateCylinder(odeHandle.space, radius, 0.01); // Very thin height
        attachGeomAndSetColliderFlags();
    }
    
    if (mode & Draw) {
        vsg::Builder builder;
        builder.options = options;
        vsg::GeometryInfo geom;
        geom.position = {0.0f, 0.0f, 0.0f};
        geom.dx = {radius, 0.0f, 0.0f};
        geom.dy = {0.0f, radius, 0.0f};
        geom.dz = {0.0f, 0.0f, 0.01f}; // Very thin in z direction
        geom.color = vsg::vec4(0.8f, 0.8f, 0.1f, 1.0f);

        vsg::StateInfo state;
        state.lighting = false;
        vsgPrimitive = builder.createDisk(geom, state);
        color = Color(geom.color);
        
        transformNode = vsg::MatrixTransform::create();
        transformNode->matrix = vsg::dmat4(1.0f);
        transformNode->addChild(vsgPrimitive);
        
        if (vsgHandle.cfg && vsgHandle.cfg->normalState) {
            transformNode->addChild(vsgHandle.cfg->normalState);
        }

        if (vsgHandle.parent) {
            vsgHandle.parent->addChild(transformNode);
        }
    }
}

void Disk::update() {
    if (mode & Draw && transformNode) {
        try {
            transformNode->matrix = getPose().getMatrix();
        } catch (const std::exception& e) {
            std::cerr << "Disk::update: " << e.what() << std::endl;
        }
    }
}

vsg::ref_ptr<vsg::MatrixTransform> Disk::getTransformNode() {
    return transformNode;
}

void Disk::setMass(double mass, bool density) {
    if (!body) return;
    
    dMass m;
    if (density) {
        // Approximate disk mass with thin cylinder
        dMassSetCylinder(&m, mass, 3, radius, 0.01);
    } else {
        dMassSetCylinderTotal(&m, mass, 3, radius, 0.01);
    }
    dBodySetMass(body, &m);
}

/******************************************************************************/
// Helper function to create Lines from vertices 
vsg::ref_ptr<vsg::Node> createLines(const std::vector<vsg::dvec3>& points)
{
    // Create vertex array
    auto vertices = vsg::vec3Array::create(points.size());
    
    // Create normals array (required by flat shaded shader)
    auto normals = vsg::vec3Array::create(points.size());
    
    // Create texture coordinates array (required by flat shaded shader)
    auto texcoords = vsg::vec2Array::create(points.size());
    
    // Create colors array
    auto colors = vsg::vec4Array::create(points.size());

    // Fill arrays
    for (size_t i = 0; i < points.size(); ++i) {
        vertices->set(i, vsg::vec3(points[i].x, points[i].y, points[i].z));
        normals->set(i, vsg::vec3(0.0f, 0.0f, 1.0f));  // Default normal pointing in Z direction
        texcoords->set(i, vsg::vec2(0.0f, 0.0f));      // Default texture coordinates
        colors->set(i, vsg::vec4(1.0f, 1.0f, 0.0f, 1.0f));  // Yellow color
    }

    // Create the vertex index draw
    auto drawCommand = vsg::VertexIndexDraw::create();
    
    // Assign arrays - order matters and must match the shader expectations
    vsg::DataList arrays;
    arrays.push_back(vertices);   // location = 0
    arrays.push_back(normals);    // location = 1
    arrays.push_back(texcoords);  // location = 2
    arrays.push_back(colors);     // location = 3
    drawCommand->assignArrays(arrays);
    
    // Create indices for LINE_STRIP
    auto indices = vsg::ushortArray::create(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        indices->set(i, static_cast<uint16_t>(i));
    }
    drawCommand->assignIndices(indices);
    drawCommand->indexCount = static_cast<uint32_t>(indices->size());
    drawCommand->instanceCount = 1;

    // Create shader set and pipeline configurator
    auto shaderSet = vsg::createFlatShadedShaderSet();
    auto pipelineConfig = vsg::GraphicsPipelineConfigurator::create(shaderSet);

    // Enable required arrays with correct strides
    pipelineConfig->enableArray("vsg_Vertex", VK_VERTEX_INPUT_RATE_VERTEX, sizeof(vsg::vec3));
    pipelineConfig->enableArray("vsg_Normal", VK_VERTEX_INPUT_RATE_VERTEX, sizeof(vsg::vec3));
    pipelineConfig->enableArray("vsg_TexCoord0", VK_VERTEX_INPUT_RATE_VERTEX, sizeof(vsg::vec2));
    pipelineConfig->enableArray("vsg_Color", VK_VERTEX_INPUT_RATE_VERTEX, sizeof(vsg::vec4));

    // Configure pipeline states
    struct SetPipelineStates : public vsg::Visitor
    {
        void apply(vsg::Object& object) override { object.traverse(*this); }
        
        void apply(vsg::InputAssemblyState& ias) override 
        {
            ias.topology = VK_PRIMITIVE_TOPOLOGY_LINE_STRIP;
        }
        
        void apply(vsg::RasterizationState& rs) override
        {
            rs.cullMode = VK_CULL_MODE_NONE;
            rs.lineWidth = 1.0f;
        }
    } sps;
    
    pipelineConfig->accept(sps);
    
    // Initialize the pipeline configuration
    pipelineConfig->init();

    // Create state group and assign pipeline
    auto stateGroup = vsg::StateGroup::create();
    pipelineConfig->copyTo(stateGroup);
    stateGroup->addChild(drawCommand);

    return stateGroup;
}

/******************************************************************************/
// Helper function to create a simple line node
// The line extends from -length/2 to +length/2 along the Z-axis.
vsg::ref_ptr<vsg::Node> createLineNode(float length) {
    std::vector<vsg::dvec3> points;
    points.push_back(vsg::dvec3(0.0, 0.0, -length * 0.5));
    points.push_back(vsg::dvec3(0.0, 0.0,  length * 0.5));
    return createLines(points);
}

// ---------------- Ray Implementation ----------------------------------------
// visualize ray, positions it, the ray's start aligns with ODE ray's position and extends along local Z-axis
Ray::Ray(double range, float thickness, float length)
    : Primitive(), range(range), thickness(thickness), length(length) {
}

Ray::~Ray() {
    // Cleanup handled by base class and ODE
}

void Ray::init(const OdeHandle& odeHandle, double mass,
               const VsgHandle& vsgHandle, char mode) {
    assert(!(mode & Body) && (mode & Geom)); 
    // Ray in ODE typically doesn't have a body, just a geom
    // A ray is used for collision detection, not for physical simulation.
    // Hence no body and mass are usually needed.

    if (!substanceManuallySet)
        substance = odeHandle.substance;
    this->mode = mode;

    // Create the ODE ray
    geom = dCreateRay(odeHandle.space, range);
    attachGeomAndSetColliderFlags();

    if (mode & Draw) {
        // If thickness == 0, use a line; else use a box to visualize the ray
        vsg::ref_ptr<vsg::Node> shapeNode;
        if (thickness == 0.0f) {
            // Create a line visualization
            shapeNode = createLineNode(length);
        } else {
            // Create a box visualization
            vsg::Builder builder;
            builder.options = options; // use VSG options

            vsg::GeometryInfo geomInfo;
            float dx = thickness * 0.5f;
            float dy = thickness * 0.5f;
            float dz = length * 0.5f;
            geomInfo.position = {0.0f, 0.0f, 0.0f};  
            geomInfo.dx = {dx, 0.0f, 0.0f};
            geomInfo.dy = {0.0f, dy, 0.0f};
            geomInfo.dz = {0.0f, 0.0f, dz};
            geomInfo.color = vsg::vec4(1.0f, 1.0f, 0.0f, 1.0f); // Yellow

            vsg::StateInfo state;
            state.lighting = false;  // Disable lighting for a simple flat color
            shapeNode = builder.createBox(geomInfo, state);
        }

        vsgPrimitive = shapeNode;
        color = Color(1.0f, 1.0f, 0.0f, 1.0f); // Yellow by default

        transformNode = vsg::MatrixTransform::create();
        transformNode->matrix = vsg::dmat4(1.0f);
        transformNode->addChild(vsgPrimitive);

        // Apply state if provided
        if (vsgHandle.cfg && vsgHandle.cfg->normalState) {
            transformNode->addChild(vsgHandle.cfg->normalState);
        }

        // add transformed primitive to the parent group in vsgHandle
        if (vsgHandle.parent) {
            vsgHandle.parent->addChild(transformNode);
        }
    }
}

void Ray::setLength(float len) {
    length = len;
    if (mode & Draw) {
        // Rebuild the geometry with the new length
        vsg::ref_ptr<vsg::Node> newShape;
        if (thickness == 0.0f) {
            // Line mode
            newShape = createLineNode(length);
        } else {
            // Box mode
            vsg::Builder builder;
            builder.options = options; // use VSG options

            float dx = thickness * 0.5f;
            float dy = thickness * 0.5f;
            float dz = length * 0.5f;

            vsg::GeometryInfo geomInfo;
            geomInfo.position = {0.0f, 0.0f, 0.0f};  
            geomInfo.dx = {dx, 0.0f, 0.0f};
            geomInfo.dy = {0.0f, dy, 0.0f};
            geomInfo.dz = {0.0f, 0.0f, dz};
            geomInfo.color = vsg::vec4(1.0f, 1.0f, 0.0f, 1.0f); // Yellow

            vsg::StateInfo state;
            state.lighting = false;  
            newShape = builder.createBox(geomInfo, state);
        }

        // Replace old primitive with new one
        if (transformNode && vsgPrimitive) {
            // TODO: Need to properly delete the old primitive
            // transformNode->removeChild(vsgPrimitive);
            vsgPrimitive = newShape;
            transformNode->addChild(vsgPrimitive);
        }
    }
}

void Ray::update() {
    if (mode & Draw && transformNode) {
        try {
            // The ODE ray is defined starting at its position and extending along the geom's local Z axis.
            // We have the line/box centered at the origin extending from -length/2 to +length/2.
            // To make it start at the ray position and extend forward, we translate it by length/2 along Z.
            
            vsg::dmat4 poseMat = getPose().getMatrix();
            vsg::dmat4 translateMat = vsg::translate(0.0, 0.0, length * 0.5);
            transformNode->matrix = poseMat * translateMat;
        } catch (const std::exception& e) {
            std::cerr << "Error in Ray::update(): " << e.what() << std::endl;
        }
    }
}

void Ray::setMass(double mass, bool density) {
    // Rays are collision detection-only objects in ODE and do not have mass or bodies.
    // This function is not applicable, but provided for interface completeness.
}


/**********************************************************************
 * Mesh Implementation
 **********************************************************************/

Mesh::Mesh(const std::string& filename, float scale)
  : filename(filename), scale(scale), boundshape(nullptr) {
}

Mesh::~Mesh(){
    if(boundshape) {
        delete boundshape;
        boundshape = nullptr;
    }
}

void Mesh::init(const OdeHandle& odeHandle, double mass,
                const VsgHandle& vsgHandle, char mode) {
    if (!substanceManuallySet)
        substance = odeHandle.substance;
    this->mode = mode;

    // Load the mesh using VSG
    // Set up options
    if(!options) {
        options = vsg::Options::create();
        options->sharedObjects = vsg::SharedObjects::create();
        options->fileCache = vsg::getEnv("VSG_FILE_CACHE");
        options->paths = vsg::getEnvPaths("VSG_FILE_PATH");
    }

    vsg::ref_ptr<vsg::Object> loadedNode = vsg::read(filename, options);
    if(!loadedNode) {
        // If cannot load, fallback to a simple box
        std::cerr << "Mesh::init: Could not load mesh from " << filename << ", using fallback box." << std::endl;
        vsg::Builder builder;
        builder.options = options;
        vsg::GeometryInfo geom;
        geom.position = {0.0f, 0.0f, 0.0f};
        geom.dx = {0.5f*scale, 0.0f, 0.0f};
        geom.dy = {0.0f, 0.5f*scale, 0.0f};
        geom.dz = {0.0f, 0.0f, 0.5f*scale};
        geom.color = vsg::vec4(0.8f,0.0f,0.8f,1.0f);

        vsg::StateInfo state;
        state.lighting = false;
        meshNode = builder.createBox(geom, state);
    } else {
        // If loaded successfully, apply scaling if needed
        meshNode = loadedNode.cast<vsg::Node>();
        if(scale != 1.0f) {
            auto scaleNode = vsg::MatrixTransform::create();
            scaleNode->matrix = vsg::scale(scale, scale, scale);
            scaleNode->addChild(meshNode);
            meshNode = scaleNode;
        }
    }

    // Compute bounding sphere
    vsg::ComputeBounds cb;
    meshNode->accept(cb);
    auto bounds = cb.bounds;
    double radius = std::max({bounds.max.x - bounds.min.x, bounds.max.y - bounds.min.y, bounds.max.z - bounds.min.z}) * 0.5;
    if(radius <= 0) radius = 0.01; // fallback

    // Create transform node for the mesh
    transformNode = vsg::MatrixTransform::create();
    transformNode->matrix = vsg::dmat4(1.0);
    transformNode->addChild(meshNode);

    // add the transformNode to the vsgHandle parent if drawing
    if ((mode & Draw) && vsgHandle.parent) {
        vsgHandle.parent->addChild(transformNode);
    }

    // If we have a body
    if (mode & Body) {
        body = dBodyCreate(odeHandle.world);
        setMass(mass, mode & Density);
    }

    // If we have geometry
    if (mode & Geom) {
        // Attempt to load bounding shape (filename+".bbox") or fallback
        // Here we simulate bounding shape. If bounding shape is not found,
        // fallback to a sphere:
        // In a real scenario, you'd implement a proper bounding shape loader.

        // fallback bounding shape:
        if(!boundshape) {
            // Just use a sphere as bounding shape
            Primitive* bound = new Sphere(radius);
            Transform* trans = new Transform(this, bound, vsg::translate(0.0,0.0,0.0));
            trans->init(odeHandle, 0, vsgHandle.changeColor(Color(1,0,0,0.3)), Geom | (vsgHandle.drawBoundings ? Draw : 0));
            boundshape = trans; // store this as boundshape
        }
    }
}

void Mesh::update(){
    if (mode & Draw) {
        try {
            vsg::dmat4 matrix = getPose().getMatrix();
            
            if (transformNode) {
                transformNode->matrix = matrix;
            }
            
            if (boundshape) {
                boundshape->setMatrix(matrix);
            }
        } catch (const std::exception& e) {
            std::cerr << "Mesh::update: " << e.what() << std::endl;
        }
    }
}

void Mesh::setMass(double mass, bool density){
    if(body){
        // simple approximation: use a sphere mass distribution
        dMass m;
        float r = getRadius();
        if (density) {
            dMassSetSphere(&m, mass, r);
        } else {
            dMassSetSphereTotal(&m, mass, r);
        }
        dBodySetMass(body, &m);
    }
}

float Mesh::getRadius() {
    if(!meshNode) return 0.01f;
    // compute again if needed
    vsg::ComputeBounds cb;
    meshNode->accept(cb);
    auto bounds = cb.bounds;
    float r = (float)(std::max({bounds.max.x - bounds.min.x, bounds.max.y - bounds.min.y, bounds.max.z - bounds.min.z}) * 0.5);
    return (r>0) ? r : 0.01f;
}

void Mesh::setBoundingShape(Primitive* boundingShape) {
    if (boundshape) {
        delete boundshape;
        boundshape = nullptr;
    }
    boundshape = boundingShape;
}

void Mesh::setPose(const Pose& pose) {
    if (body) {
        dMatrix3 R;
        odeRotation(pose, R);
        dBodySetPosition(body, pose[3][0], pose[3][1], pose[3][2]);
        dBodySetRotation(body, R);
    } else if (geom) {
        dMatrix3 R;
        odeRotation(pose, R);
        dGeomSetPosition(geom, pose[3][0], pose[3][1], pose[3][2]);
        dGeomSetRotation(geom, R);
    } else {
        // store for later
        poseWithoutBodyAndGeom = pose;
    }
    update();
}

/**********************************************************************
 * Transform Implementation
 **********************************************************************/

Transform::Transform(Primitive* parent, Primitive* child, const Pose& pose, bool deleteChild)
  : parent(parent), child(child), pose(pose), deleteChild(deleteChild) {
}

Transform::~Transform(){
    if(child && deleteChild)
        delete child;
}

void Transform::init(const OdeHandle& odeHandle, double mass,
                     const VsgHandle& vsgHandle,
                     char mode) {
    assert(parent && parent->getBody() != 0 && child);
    assert(child->getBody() == 0 && child->getGeom() == 0); // child should not be initialised yet
    this->mode = mode | Primitive::_Transform;
    if (!substanceManuallySet)
        substance = odeHandle.substance;

    // ODE transform geom
    geom = dCreateGeomTransform(odeHandle.space);
    dGeomTransformSetInfo(geom, 1);
    dGeomTransformSetCleanup(geom, 0);

    // The child must be initialised with no body bit set
    OdeHandle odeHandleChild(odeHandle);
    odeHandleChild.space = 0; // child inherits space from transform
    VsgHandle vsgHandleChild(vsgHandle);
    // If parent has a transformNode, use it as a parent node for child's draw
    vsgHandleChild.parent = parent->getTransformNode();

    child->init(odeHandleChild, mass, vsgHandleChild, (mode & ~Body) | _Child);
    child->setPose(pose);

    dGeomTransformSetGeom(geom, child->getGeom());
    // Attach transform to parent's body
    dGeomSetBody(geom, parent->getBody());
    dGeomSetData(geom, (void*)this);
    body = parent->getBody(); // same body as parent

    // Create a transform node for visualization (if needed)
    // Actually, child already has its visualization. Our transform doesn't add another node,
    // Because child's position is set via ODE transform. If you want a separate node, you can add one,
    // but it's not strictly necessary if child's node is already positioned relative to parent's node.
    // We can create a transformNode that just passes child's node through.
    // However, since child->init created its own transform, that should suffice.

    // Just store a reference to child's transform node if we need a getTransformNode()
    transformNode = vsg::MatrixTransform::create();
    transformNode->matrix = vsg::dmat4(1.0);
    // The child's transform node is part of child's structure, we just reuse child's visual scene:
    if (child->getTransformNode()) {
        transformNode->addChild(child->getTransformNode());
    }
    if ((mode & Draw) && vsgHandle.parent) {
        vsgHandle.parent->addChild(transformNode);
    }
}

void Transform::update(){
    if(child) child->update();
    // The transform is handled by ODE, child's pose is already correct
    // If needed, we could update transformNode->matrix here, but since child->setPose(pose)
    // sets child's relative pose and parent's body sets global pose, it's all handled by ODE updates.
}

void Transform::setMass(double mass, bool density){
    if(child) child->setMass(mass, density);
}

/**********************************************************************
 * DummyPrimitive Implementation
 **********************************************************************/

// Already defined in the header, implementation trivial
// Just ensure no ODE or drawing is done.
// No changes needed beyond what is defined in the header and the base class methods.




}