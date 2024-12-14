// base.h

#ifndef __BASE_H
#define __BASE_H

#include <ode/ode.h>
#include <vsg/all.h>

#include "vsghandle.h"
#include "odehandle.h"
#include "configurable.h"

namespace lpzrobots {

    struct StatLineProperties {
        // Default constructor
        StatLineProperties()
            : fontSizeTime(12), fontSizeText(12), fontColor("white") {}

        StatLineProperties(int fontSizeTime, int fontSizeText, const std::string& fontColor)
            : fontSizeTime(fontSizeTime), fontSizeText(fontSizeText), fontColor(fontColor) {
        }
        int fontSizeTime;
        int fontSizeText;
        std::string fontColor;
    };

    class Plane; // Forward declaration of your Plane class

    class Base : public Configurable
    {
    public:
        Base(const std::string& caption = "LpzRobots Simulator (Martius et al)");

        static const int PHYSICS_CALLBACKABLE = 1; //!< called each ode/physics step
        static const int GRAPHICS_CALLBACKABLE = 2; //!< called each vsg/draw step

        virtual void makePhysicsScene();

        virtual void makeScene(VsgScene* scene, const VsgConfig& config);

        virtual vsg::ref_ptr<vsg::Node> makeSky(const VsgConfig& config);

        virtual vsg::ref_ptr<vsg::Node> makeGround(const VsgConfig& config);

        virtual vsg::ref_ptr<vsg::Node> createHUD(VsgScene* scene, const VsgConfig& config);

        virtual void makeLights(vsg::ref_ptr<vsg::Group> node, const VsgConfig& config);

        virtual vsg::ref_ptr<vsg::Node> createShadowedScene(vsg::ref_ptr<vsg::Node> sceneToShadow,
                                                    vsg::ref_ptr<vsg::Light> lightSource,
                                                    int shadowType);


        virtual void setGroundTexture(const std::string& filename) {
            this->groundTexture = filename;
        }

        virtual Substance getGroundSubstance();

        virtual void setGroundSubstance(const Substance& substance);

        virtual void setCaption(const std::string& caption);

        virtual void setTitle(const std::string& title);

        virtual StatLineProperties getStatLineProperties() { return statlineprop; }

        virtual void setStatLineProperties(const StatLineProperties& statlineprop) {
            this->statlineprop = statlineprop;
        }

        virtual ~Base();

    protected:
        virtual void setTimeStats(double time, double realtimefactor,
                                  double truerealtimefactor, bool pause);

        virtual void changeShadowTechnique();

        virtual void base_close();

        // ODE ground plane
        dGeomID ground;

        VsgHandle vsgHandle;
        OdeHandle odeHandle;

        std::string caption;
        std::string title;

        std::string groundTexture;

        vsg::ref_ptr<vsg::Group> rootNode;

        vsg::ref_ptr<vsg::Node> hudNode;

        StatLineProperties statlineprop;

        Plane* plane; // Correct type for plane

        int shadowTexSize;
        bool useNVidia;

    public:
        static int contains(char** list, int len, const char* str);

    };

}

#endif // __BASE_H
