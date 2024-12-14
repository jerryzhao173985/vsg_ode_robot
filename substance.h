// substance.h - VSG Version

#ifndef __SUBSTANCE_H
#define __SUBSTANCE_H

#include <ode/ode.h>
#include <functional>

namespace lpzrobots {

    class GlobalData;
    class Substance;
    class Axis;

    /**
       Function to be called at a collision event between the two geoms.
       @param params Surface parameters, which should be changed/calculated by this function
       @param globaldata Global information
       @param userdata Pointer to user data for this callback (stored in substance)
       @param contacts Array of contact information
       @param numContacts Length of contact information array
       @param o1 Geom corresponding to substance of this callback
       @param o2 Other geom
       @param s1 Substance of this callback
       @param s2 Other substance
       @return 0 if collision should not be treated;
               1 if collision should be treated otherwise (by other callback or standard methods);
               2 if collision to be treated and parameters for collision are set in params
     */
    typedef std::function<int(dSurfaceParameters& params, GlobalData& globaldata, void* userdata,
                              dContact* contacts, int numContacts,
                              dGeomID o1, dGeomID o2, const Substance& s1, const Substance& s2)>
        CollisionCallback;

    /**
       Physical substance definition, used for collision detection/treatment.
       This class defines the physical properties of a material (substance) in the simulation.
    */
    class Substance {
    public:
        Substance();
        Substance(float roughness, float slip, float hardness, float elasticity);

    public:
        float roughness;
        float slip;
        float hardness;
        float elasticity;

        void setCollisionCallback(CollisionCallback func, void* userdata);

        CollisionCallback callback;
        void* userdata;

    public:
        /// Combination of two surfaces
        static void getSurfaceParams(dSurfaceParameters& sp, const Substance& s1, const Substance& s2, double stepsize);

        static void printSurfaceParams(const dSurfaceParameters& surfParams);

        //// Factory methods

        /// Default substance is plastic with roughness=0.8
        static Substance getDefaultSubstance();
        void toDefaultSubstance();

        /// Very hard and elastic with slip, roughness [0.1-1]
        static Substance getMetal(float roughness);
        /// Very hard and elastic with slip, roughness [0.1-1]
        void toMetal(float roughness);

        /// High roughness, no slip, very elastic, hardness [5-50]
        static Substance getRubber(float hardness);
        /// High roughness, no slip, very elastic, hardness [5-50]
        void toRubber(float hardness);

        /// Medium slip, a bit elastic, medium hardness, roughness [0.5-2]
        static Substance getPlastic(float roughness);
        /// Medium slip, a bit elastic, medium hardness, roughness [0.5-2]
        void toPlastic(float roughness);

        /// Large slip, not elastic, low hardness [1-30], high roughness
        static Substance getFoam(float hardness);
        /// Large slip, not elastic, low hardness [1-30], high roughness
        void toFoam(float hardness);

        /** Variable slip and roughness [0-1], not elastic, high hardness for solid snow
            slip = 1 <--> roughness=0.0, slip = 0 <--> roughness=1.0 */
        static Substance getSnow(float slip);
        /** Variable slip and roughness, not elastic, high hardness for solid snow
            slip = 1 <--> roughness=0.0, slip = 0 <--> roughness=1.0 */
        void toSnow(float slip);

        /// @see toNoContact()
        static Substance getNoContact();
        /** Set the collision callback to ignore everything.
            Usually, it is better to use the "ignorePairs" from OdeHandle, but
            if this particular substance should not collide with any other, this is easier.
            WARNING: this sets the collisionCallback. This will not convert to other
            substances without manually setting the callback to nullptr.
         */
        void toNoContact();

        /** Enables anisotropic friction.
            The friction along the given axis is ratio-fold of the friction in the other directions.
            If ratio = 0.1 and axis=Axis(0,0,1) then the friction along the z-axis
             is 1/10th of the normal friction.
            Useful to mimic scales of snakes or the like.
            WARNING: this sets the collisionCallback!
            To disable, the collisionCallback has to be set to nullptr manually.
        */
        void toAnisotropicFriction(double ratio, const Axis& axis);
    };

    class DebugSubstance : public Substance {
    public:
        DebugSubstance();
        DebugSubstance(float roughness, float slip, float hardness, float elasticity);
    protected:
        static int dbg_output(dSurfaceParameters& params, GlobalData& globaldata, void* userdata,
                              dContact* contacts, int numContacts,
                              dGeomID o1, dGeomID o2, const Substance& s1, const Substance& s2);
    };

}

#endif // __SUBSTANCE_H
