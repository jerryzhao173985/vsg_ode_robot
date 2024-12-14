// vsghandle.h - VSG Version
// Converted from osghandle.h

/***************************************************************************
 *   Original file: osghandle.h                                            *
 *   Converted to use Vulkan Scene Graph (VSG)                             *
 *                                                                         *
 *   Note: This code is a conversion of an OSG-based application to use    *
 *   VSG. Some functionalities may not have direct equivalents in VSG and  *
 *   may need custom implementation.                                       *
 ***************************************************************************/

#ifndef __VSGHANDLE_H
#define __VSGHANDLE_H

#include <vsg/all.h>
#include "color.h"
#include "colorschema.h"
#include "odehandle.h"

namespace lpzrobots {

    // class RobotCameraManager;

    /** Data structure containing some configuration variables for VSG */
    struct VsgConfig {
        VsgConfig() : normalState(nullptr), transparentState(nullptr), cs(nullptr), 
                      noGraphics(false) {}
        ~VsgConfig() {} // Don't clean up here - let VsgHandle handle it
        
        ColorSchema* colorSchema() {return cs;}

        // VSG does not have TessellationHints; this would need to be implemented if required
        // vsg::ref_ptr<vsg::TessellationHints> tesselhints[3];

        vsg::ref_ptr<vsg::StateGroup> normalState;
        vsg::ref_ptr<vsg::StateGroup> transparentState;
        // vsg::ref_ptr<vsg::Node> normalState;
        // vsg::ref_ptr<vsg::Node> transparentState;

        ColorSchema* cs; // Color schema
        bool noGraphics;
    };

    /** Data structure containing the scene nodes */
    struct VsgScene {
        VsgScene()
            : root(nullptr), world(nullptr), scene(nullptr),
              groundScene(nullptr), lightSource(nullptr), worldTransform(nullptr)
              {}
        ~VsgScene() { clear(); }
        void clear();
        bool isValid() const;

        vsg::ref_ptr<vsg::Group> root;       // Master node (contains world, HUD, etc.)
        vsg::ref_ptr<vsg::Group> world;      // World node (contains ground, sky, and scene)
        vsg::ref_ptr<vsg::Group> scene;      // Actual scene for robots and objects

        vsg::ref_ptr<vsg::Node> groundScene;

        vsg::ref_ptr<vsg::Light> lightSource;          // The light source
        vsg::ref_ptr<vsg::MatrixTransform> worldTransform; // World transformation

        // RobotCameraManager* robotCamManager; // Manages robot cameras and their display
    };

    /** Data structure for accessing the Vulkan Scene Graph */
    class VsgHandle
    {
    public:
        VsgHandle();

        ~VsgHandle();

        // Make VsgHandle Non-Copyable: If copying VsgHandle leads to multiple 
        // destructors cleaning the same resources, delete the copy constructor 
        // and copy assignment operator:
        // VsgHandle(const VsgHandle&) = delete;
        // VsgHandle& operator=(const VsgHandle&) = delete;

        /// Initialization of the structure
        void init();
        /// Set up robot camera manager (must be called after init but before usage of the structure)
        // void setup(int windowW, int windowH);
        /// Deletes all internal variables
        void close();

        /// Decides whether to draw bounding boxes
        bool drawBoundings;

        Color color;

        VsgConfig* cfg;    // The config is shared
        VsgScene* scene;   // The scene is shared
        vsg::ref_ptr<vsg::Group> parent; // The place where individual VSG primitives are added

        // /// Returns a new VsgHandle with only the color changed
        // VsgHandle& changeColor(const Color& color);
        // /// Returns a new VsgHandle with only the color changed
        // VsgHandle& changeColor(double r, double g, double b, double a = 1.0);
        // /// Returns a new VsgHandle with only the alpha channel changed
        // VsgHandle& changeAlpha(double alpha);

        // /** Returns a new VsgHandle with only the color changed
        //     @param name Name, ID, or alias of a color in the color schema
        //     The current color_set is used
        //  */
        // VsgHandle& changeColor(const std::string& name);
        /// returns a new osghandle with only the color changed
        VsgHandle changeColor(const Color& color) const;
        /// returns a new osghandle with only the color changed
        VsgHandle changeColor(double r, double g, double b, double a=1.0) const;
        /// returns a new osghandle with only the alpha channel changed
        VsgHandle changeAlpha(double alpha) const; 

        /** returns a new osghandle with only the color changed
             @param name name,id, or alias of a color in the colorschema
            The current color_set is used
        */
        VsgHandle changeColor(const std::string& name) const;

        /** Like changeColor(string) but with a default color (defcolor) in case
            no color with the name exists */
        VsgHandle& changeColorDef(const std::string& name, const Color& defcolor);

        /** Returns the color that corresponds to the name (name, ID, or alias)
            in the color schema. The current color_set is used
        */
        Color getColor(const std::string& name) const;

        /** Returns a new VsgHandle with a changed color (alias) set */
        VsgHandle& changeColorSet(int color_set);

        /// Modifies the used color set. Only applies to new set colors.
        void setColorSet(int color_set);

        /** Returns the color schema. Use this to set/load colors and aliases
            Note, the color schema is shared among the VsgHandles
         */
        ColorSchema* colorSchema();
        const ColorSchema* colorSchema() const;

    private:
        int color_set; // Selects the color (alias) set that is used when setting a color
        bool hasBeenClosed;  // Add this flag
    };

}

#endif // __VSGHANDLE_H
