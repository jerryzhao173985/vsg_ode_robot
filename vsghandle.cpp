// vsgHandle.cpp - VSG Version
// Converted from osghandle.cpp

#include "vsghandle.h"
// #include "robotcameramanager.h"
#include <vsg/all.h>
#include <iostream>

namespace lpzrobots {

    VsgHandle::VsgHandle()
        : drawBoundings(false), cfg(nullptr), scene(nullptr), parent(nullptr), color_set(0),
          color(1.0f, 1.0f, 1.0f, 1.0f), hasBeenClosed(false) {
    }

    
    VsgHandle::~VsgHandle() {
        // // Only call close() if it has not been called yet
        // if (!hasBeenClosed) {
        //     close();
        // }
    }


    
    void VsgHandle::init() {
        // Initialize the scene first
        if (!scene) {
            scene = new VsgScene();
            
            // Create node hierarchy with validation
            scene->root = vsg::Group::create();
            if (!scene->root) {
                throw std::runtime_error("Failed to create root node");
            }
            
            scene->world = vsg::Group::create();
            if (!scene->world) {
                throw std::runtime_error("Failed to create world node");
            }
            
            scene->scene = vsg::Group::create();
            if (!scene->scene) {
                throw std::runtime_error("Failed to create scene node");
            }
            
            // Build scene graph hierarchy
            scene->root->addChild(scene->world);
            scene->world->addChild(scene->scene);
            
            // Set parent reference
            parent = scene->scene;
        }

        // Initialize config
        if (!cfg) {
            cfg = new VsgConfig();
            
            // Create state groups
            cfg->normalState = vsg::StateGroup::create();
            cfg->transparentState = vsg::StateGroup::create();
            
            // Initialize color schema
            cfg->cs = new ColorSchema();

            // // Configure blending for transparency
            // auto blendState = vsg::BlendState::create();
            // cfg->transparentState->add(blendState);
            // VSG does not have TessellationHints; implement if needed
        }
    }

    // void VsgHandle::setup(int windowW, int windowH) {
    //     // Implement RobotCameraManager for VSG if required
    //     scene->robotCamManager = new RobotCameraManager(windowW, windowH);
    // }

    
    void VsgHandle::close() {
        if (hasBeenClosed) return;
        hasBeenClosed = true;
        
        // Clear parent reference first
        if (parent.valid()) {
            parent = nullptr;
        }
        
        // Then clear the scene (which cleans up all VSG nodes)
        if (scene) {
            scene->clear();
            delete scene;
            scene = nullptr;
        }

        // Finally clear the configuration
        if (cfg) {
            if (cfg->cs) {
                delete cfg->cs;
                cfg->cs = nullptr;
            }
            cfg->normalState = nullptr;
            cfg->transparentState = nullptr;
            delete cfg;
            cfg = nullptr;
        }
    }


    // Now all these methods modify the current object in place and return *this.
    // They are no longer const, as they change the object's state.

    // VsgHandle& VsgHandle::changeColor(const Color& newColor) {
    //     color = newColor;
    //     return *this;
    // }

    // VsgHandle& VsgHandle::changeColor(double r, double g, double b, double a) {
    //     color = Color(r, g, b, a);
    //     return *this;
    // }

    // VsgHandle& VsgHandle::changeAlpha(double alpha) {
    //     color.alpha() = alpha;
    //     return *this;
    // }

    // VsgHandle& VsgHandle::changeColor(const std::string& name) {
    //     if (cfg && cfg->colorSchema()) {
    //         color = cfg->colorSchema()->color(name, color_set);
    //     }
    //     return *this;
    // }
    // returns a new copy of vsghandle with only the color changed
    VsgHandle VsgHandle::changeColor(const Color& color) const {
        VsgHandle copy(*this);
        copy.color = color;
        return copy;
    }

    VsgHandle VsgHandle::changeColor(double r, double g, double b, double a) const {
        VsgHandle copy(*this);
        copy.color = Color(r,g,b,a);
        return copy;
    }

    VsgHandle VsgHandle::changeAlpha(double alpha) const {
        VsgHandle copy(*this);
        copy.color.alpha() = alpha;
        return copy;
    }

    VsgHandle VsgHandle::changeColor(const std::string& name) const {
        VsgHandle copy(*this);
        if(cfg && cfg->cs)
        copy.color = cfg->cs->color(name,color_set);
        return copy;
    }

    Color VsgHandle::getColor(const std::string& name) const {
        if (cfg && cfg->colorSchema())
            return cfg->colorSchema()->color(name, color_set);
        else {
            return Color();
        }
    }

    VsgHandle& VsgHandle::changeColorDef(const std::string& name, const Color& defcolor) {
        if (cfg && cfg->colorSchema()) {
            if (!cfg->colorSchema()->color(this->color, name, color_set)) {
                this->color = defcolor;
            }
        }
        return *this;
    }

    VsgHandle& VsgHandle::changeColorSet(int new_color_set) {
        if (new_color_set >= 0)
            this->color_set = new_color_set;
        return *this;
    }

    ColorSchema* VsgHandle::colorSchema() {
        return cfg ? cfg->colorSchema() : nullptr;
    }

    const ColorSchema* VsgHandle::colorSchema() const {
        return cfg ? cfg->colorSchema() : nullptr;
    }
    

    void VsgHandle::setColorSet(int new_color_set) {
        if (new_color_set >= 0)
            this->color_set = new_color_set;
    }

    void VsgScene::clear() {
        if (!root && !world && !scene) {
            return; // Already cleared
        }

        // Clear scene first (bottom-up cleanup)
        if (scene) {
            if (auto sceneGroup = scene.cast<vsg::Group>()) {
                sceneGroup->children.clear();
            }
            scene = nullptr;
        }

        // Clear world
        if (world) {
            if (auto worldGroup = world.cast<vsg::Group>()) {
                worldGroup->children.clear();
            }
            world = nullptr;
        }

        // Clear root
        if (root) {
            if (auto rootGroup = root.cast<vsg::Group>()) {
                rootGroup->children.clear();
            }
            root = nullptr;
        }

        // Clear ground scene
        groundScene = nullptr;
        
        // Clear lighting
        lightSource = nullptr;
        
        // Clear world transform
        worldTransform = nullptr;
    }


    bool VsgScene::isValid() const {
        return root.valid() && world.valid() && scene.valid();
    }

    void VsgHandle::setupLighting(const vsg::dvec3& direction, const vsg::vec4& color) {
        if (!scene || !scene->world) {
            std::cerr << "VsgHandle::setupLighting: Scene not initialized, call init() first" << std::endl;
            return;
        }
        
        // Create a directional light
        auto light = vsg::DirectionalLight::create();
        light->name = "main_light";
        light->intensity = 1.0f;
        light->direction = vsg::normalize(direction);
        
        // Set the color (handle differently to avoid type errors)
        light->color.set(color.x, color.y, color.z); // Only use RGB components
        
        // Create a transform for the light
        auto lightTransform = vsg::MatrixTransform::create();
        lightTransform->matrix = vsg::lookAt(vsg::dvec3(0.0, 0.0, 0.0), direction, vsg::dvec3(0.0, 1.0, 0.0));
        
        // Add the light to the transform
        lightTransform->addChild(light);
        
        // Add the light transform to the world
        scene->world->addChild(lightTransform);
        
        // Store the light for later access
        scene->lightSource = light;
    }

}
