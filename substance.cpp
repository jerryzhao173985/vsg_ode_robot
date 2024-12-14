// substance.cpp - VSG Version

#include "substance.h"
#include "globaldata.h"
#include "axis.h"
#include "pose.h"
#include "primitive.h"
#include <iostream>
#include <cassert>

namespace lpzrobots {

    Substance::Substance()
        : roughness(0.8f), slip(0.01f), hardness(40.0f), elasticity(0.5f),
          callback(nullptr), userdata(nullptr)
    {
    }

    Substance::Substance(float roughness, float slip, float hardness, float elasticity)
        : roughness(roughness), slip(slip), hardness(hardness), elasticity(elasticity),
          callback(nullptr), userdata(nullptr)
    {
    }

    void Substance::setCollisionCallback(CollisionCallback func, void* userdata_) {
        callback = func;
        userdata = userdata_;
    }

    // Combination of two surfaces
    void Substance::getSurfaceParams(dSurfaceParameters& sp, const Substance& s1, const Substance& s2, double stepsize) {
        sp.mu = s1.roughness * s2.roughness;

        dReal kp = 100 * s1.hardness * s2.hardness / (s1.hardness + s2.hardness);
        double kd1 = (1.00 - s1.elasticity);
        double kd2 = (1.00 - s2.elasticity);
        dReal kd = 50 * (kd1 * s2.hardness + kd2 * s1.hardness) / (s1.hardness + s2.hardness);

        sp.soft_erp = stepsize * kp / (stepsize * kp + kd);
        sp.soft_cfm = 1 / (stepsize * kp + kd);

        sp.slip1 = s1.slip + s2.slip;
        sp.slip2 = s1.slip + s2.slip;
        if (sp.slip1 < 0.0001)
            sp.mode = 0;
        else
            sp.mode = dContactSlip1 | dContactSlip2;
        sp.mode |= dContactSoftERP | dContactSoftCFM | dContactApprox1;
    }

    void Substance::printSurfaceParams(const dSurfaceParameters& sp) {
        std::cout << "Surface mu: " << sp.mu
                  << ", soft_erp: " << sp.soft_erp
                  << ", soft_cfm: " << sp.soft_cfm
                  << ", slip1: " << sp.slip1
                  << std::endl;
    }

    // Factory methods
    Substance Substance::getDefaultSubstance() {
        Substance s;
        s.toDefaultSubstance();
        return s;
    }

    void Substance::toDefaultSubstance() {
        toPlastic(0.8f);
    }

    // Very hard and elastic with slip
    Substance Substance::getMetal(float _roughness) {
        Substance s;
        s.toMetal(_roughness);
        return s;
    }

    void Substance::toMetal(float _roughness) {
        if (_roughness < 0) { std::cerr << "Negative roughness in metal!" << std::endl; }
        if (_roughness > 2) { std::cerr << "Very rough metal used!" << std::endl; }
        roughness = _roughness;
        hardness = 200;
        elasticity = 0.8f;
        slip = 0.01f;
    }

    // High roughness, no slip, very elastic, hardness : [5-50]
    Substance Substance::getRubber(float _hardness) {
        Substance s;
        s.toRubber(_hardness);
        return s;
    }

    void Substance::toRubber(float _hardness) {
        if (_hardness < 5) { std::cerr << "Too soft rubber!" << std::endl; }
        if (_hardness > 50) { std::cerr << "Too hard rubber!" << std::endl; }
        roughness = 3.0f;
        hardness = _hardness;
        elasticity = 0.95f;
        slip = 0.0f;
    }

    // Medium slip, a bit elastic, medium hardness, roughness [0.5-2]
    Substance Substance::getPlastic(float _roughness) {
        Substance s;
        s.toPlastic(_roughness);
        return s;
    }

    void Substance::toPlastic(float _roughness) {
        if (_roughness < 0) { std::cerr << "Negative roughness in plastic!" << std::endl; }
        if (_roughness > 3) { std::cerr << "Very rough plastic used!" << std::endl; }
        roughness = _roughness;
        hardness = 40.0f;
        elasticity = 0.5f;
        slip = 0.01f;
    }

    // Large slip, not elastic, low hardness [1-30], high roughness
    Substance Substance::getFoam(float _hardness) {
        Substance s;
        s.toFoam(_hardness);
        return s;
    }

    void Substance::toFoam(float _hardness) {
        if (_hardness < 1) { std::cerr << "Too soft foam!" << std::endl; }
        if (_hardness > 30) { std::cerr << "Too hard foam!" << std::endl; }
        roughness = 2.0f;
        hardness = _hardness;
        elasticity = 0.0f;
        slip = 0.1f;
    }

    // Variable slip and roughness, not elastic, high hardness for solid snow
    Substance Substance::getSnow(float _slip) {
        Substance s;
        s.toSnow(_slip);
        return s;
    }

    void Substance::toSnow(float _slip) {
        if (_slip < 0) { std::cerr << "Slip is not defined for values < 0!" << std::endl; }
        if (_slip > 1) { std::cerr << "Too high slip!" << std::endl; }
        roughness = 1.0f - _slip;
        hardness = 40.0f;
        elasticity = 0.0f;
        slip = _slip;
    }

    // No contact points are generated
    Substance Substance::getNoContact() {
        Substance s;
        s.toNoContact();
        return s;
    }

    // Collision function that does nothing and prohibits further treatment of collision event.
    static int dummyCallBack(dSurfaceParameters& params, GlobalData& globaldata, void* userdata,
                             dContact* contacts, int numContacts,
                             dGeomID o1, dGeomID o2, const Substance& s1, const Substance& s2) {
        return 0;
    }

    void Substance::toNoContact() {
        toDefaultSubstance();
        setCollisionCallback(dummyCallBack, nullptr);
    }

    // *** Anisotropic friction stuff ***

    struct AnisotropicFrictionData {
        AnisotropicFrictionData(double r = 0.0, const Axis& a = Axis()) 
            : ratio(r), axis(a) {}
        double ratio;
        Axis axis;
    };

    
    static int anisotropicCallback(dSurfaceParameters& params, GlobalData& globaldata, void* userdata,
                            dContact* contacts, int numContacts,
                            dGeomID o1, dGeomID o2, const Substance& s1, const Substance& s2) {
        // The other substance should not have a callback itself
        if (s2.callback)
            return 1;

        AnisotropicFrictionData* data = static_cast<AnisotropicFrictionData*>(userdata);
        assert(data && "Anisotropic callback does not have correct userdata!");

        // Get object pose
        const dReal* pos = dGeomGetPosition(o1);
        const dReal* rot = dGeomGetRotation(o1);
        Pose pose(pos, rot);

        // Transform axis to world coordinates
        vsg::dvec3 objectAxis = data->axis.toVec3();
        vsg::dvec4 transformedAxis(objectAxis.x, objectAxis.y, objectAxis.z, 0.0);
        transformedAxis = pose.matrix() * transformedAxis;
        objectAxis = vsg::dvec3(transformedAxis.x, transformedAxis.y, transformedAxis.z);

        for (int i = 0; i < numContacts; i++) {
            vsg::dvec3 normal(contacts[i].geom.normal[0], 
                            contacts[i].geom.normal[1], 
                            contacts[i].geom.normal[2]);
            vsg::dvec3 dir = vsg::cross(objectAxis, normal);
            if (std::isnan(dir.x) || vsg::length2(dir) < 0.1) {
                return 1; // Do normal friction
            } else {
                dir = vsg::normalize(dir);
                contacts[i].fdir1[0] = dir.x;
                contacts[i].fdir1[1] = dir.y;
                contacts[i].fdir1[2] = dir.z;
            }
        }

        // Calculate default params using step size from globaldata
        // double stepSize = globaldata.odeConfig.simStepSize;
        double stepSize = 1.0;
        Substance::getSurfaceParams(params, s1, s2, stepSize);
        
        // Set new friction parameters
        params.mu2 = params.mu * (data->ratio);
        params.mode |= dContactMu2 | dContactFDir1;

        return 2;
    }

    void Substance::toAnisotropicFriction(double ratio, const Axis& axis) {
        // Create with explicit constructor call
        AnisotropicFrictionData* data = new AnisotropicFrictionData(ratio, axis);
        setCollisionCallback(anisotropicCallback, data);
    }

    // *** End anisotropic friction stuff ***

    DebugSubstance::DebugSubstance() {
        setCollisionCallback(&dbg_output, nullptr);
    }

    DebugSubstance::DebugSubstance(float roughness, float slip, float hardness, float elasticity)
        : Substance(roughness, slip, hardness, elasticity) {
        setCollisionCallback(dbg_output, nullptr);
    }

    int DebugSubstance::dbg_output(dSurfaceParameters& params, GlobalData& globaldata, void* userdata,
                                   dContact* contacts, int numContacts,
                                   dGeomID o1, dGeomID o2,
                                   const Substance& s1, const Substance& s2) {
        dSurfaceParameters sp;
        getSurfaceParams(sp, s1, s2, 1.0 /*globaldata.odeConfig.simStepSize*/);
        printSurfaceParams(sp);

        return 1;
    }

}
