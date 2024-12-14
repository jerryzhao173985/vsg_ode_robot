// base.cpp

#include "base.h"
#include "primitive.h"
#include "pos.h"
#include "odehandle.h"
#include "globaldata.h"
#include <iostream>

namespace lpzrobots {

Base::Base(const std::string& caption)
    : caption(caption), title(""), groundTexture("Images/whiteground.jpg"),
      statlineprop(), // Default initialization
      shadowTexSize(1024), useNVidia(true),
      plane(nullptr), rootNode(nullptr), hudNode(nullptr) {
}

Base::~Base() {
    base_close();
}

void Base::base_close() {
    if (plane)
        delete plane;
}

void Base::makePhysicsScene() {
    ground = dCreatePlane(odeHandle.space, 0, 0, 1, 0);
    dGeomSetCategoryBits(ground, Primitive::Stat);
    dGeomSetCollideBits(ground, ~Primitive::Stat);

    plane = new Plane();
    dGeomSetData(ground, (void*)plane);
}

Substance Base::getGroundSubstance() {
    if (plane)
        return plane->substance;
    else
        return Substance();
}

void Base::setGroundSubstance(const Substance& substance) {
    if (plane)
        plane->setSubstance(substance);
}

void Base::makeScene(VsgScene* scene, const VsgConfig& config) {
    // Create root node
    scene->root = vsg::Group::create();

    // Create world group
    scene->world = vsg::Group::create();

    // Create scene group
    scene->scene = vsg::Group::create();

    // Create sky and ground
    auto skyNode = makeSky(config);
    auto groundNode = makeGround(config);

    // Add sky and ground to world
    scene->world->addChild(skyNode);
    scene->world->addChild(groundNode);

    // Add scene to world
    scene->world->addChild(scene->scene);

    // Add world to root
    scene->root->addChild(scene->world);

    // Create HUD
    auto hudNode = createHUD(scene, config);
    if (hudNode) {
        scene->root->addChild(hudNode);
    }
}

vsg::ref_ptr<vsg::Node> Base::makeSky(const VsgConfig& config) {
    // Parameters for the sphere
    const uint32_t longitudeSegments = 64;
    const uint32_t latitudeSegments = 32;
    const float radius = 5000.0f;

    // Create arrays
    std::vector<vsg::dvec3> vertexData;
    std::vector<vsg::dvec3> normalData;
    std::vector<vsg::vec2> texCoordData;
    std::vector<uint16_t> indexData;

    // Pre-allocate space
    vertexData.reserve((longitudeSegments + 1) * (latitudeSegments + 1));
    normalData.reserve((longitudeSegments + 1) * (latitudeSegments + 1));
    texCoordData.reserve((longitudeSegments + 1) * (latitudeSegments + 1));
    indexData.reserve(longitudeSegments * latitudeSegments * 6);

    // Generate sphere geometry
    for (uint32_t y = 0; y <= latitudeSegments; ++y) {
        for (uint32_t x = 0; x <= longitudeSegments; ++x) {
            float xSegment = static_cast<float>(x) / longitudeSegments;
            float ySegment = static_cast<float>(y) / latitudeSegments;
            float xPos = radius * std::cos(xSegment * 2.0f * M_PI) * std::sin(ySegment * M_PI);
            float yPos = radius * std::cos(ySegment * M_PI);
            float zPos = radius * std::sin(xSegment * 2.0f * M_PI) * std::sin(ySegment * M_PI);

            vertexData.push_back({xPos, yPos, zPos});
            normalData.push_back({-xPos, -yPos, -zPos}); // Inverted normals for inside view
            texCoordData.push_back({xSegment, ySegment});
        }
    }

    // Generate indices
    for (uint32_t y = 0; y < latitudeSegments; ++y) {
        for (uint32_t x = 0; x < longitudeSegments; ++x) {
            uint32_t i0 = y * (longitudeSegments + 1) + x;
            uint32_t i1 = i0 + longitudeSegments + 1;

            indexData.push_back(i0);
            indexData.push_back(i1);
            indexData.push_back(i0 + 1);

            indexData.push_back(i0 + 1);
            indexData.push_back(i1);
            indexData.push_back(i1 + 1);
        }
    }

    // Create arrays
    auto vertices = vsg::dvec3Array::create(vertexData.size(), vertexData.data());
    auto normals = vsg::dvec3Array::create(normalData.size(), normalData.data());
    auto texCoords = vsg::vec2Array::create(texCoordData.size(), texCoordData.data());
    auto indices = vsg::ushortArray::create(indexData.size(), indexData.data());

    // Create geometry
    auto geometry = vsg::Geometry::create();

    // Create vertex input bindings
    vsg::VertexInputState::Bindings vertexBindings{
        VkVertexInputBindingDescription{0, sizeof(vsg::dvec3), VK_VERTEX_INPUT_RATE_VERTEX},
        VkVertexInputBindingDescription{1, sizeof(vsg::dvec3), VK_VERTEX_INPUT_RATE_VERTEX},
        VkVertexInputBindingDescription{2, sizeof(vsg::vec2), VK_VERTEX_INPUT_RATE_VERTEX}
    };

    // Create vertex input attributes
    vsg::VertexInputState::Attributes vertexAttributes{
        VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0}, // vertices
        VkVertexInputAttributeDescription{1, 1, VK_FORMAT_R32G32B32_SFLOAT, 0}, // normals
        VkVertexInputAttributeDescription{2, 2, VK_FORMAT_R32G32_SFLOAT, 0}     // texcoords
    };

    // Create vertex input state
    auto vertexInputState = vsg::VertexInputState::create(vertexBindings, vertexAttributes);

    // Create vertex buffers
    auto vertexBuffer = vsg::BufferInfo::create();
    vertexBuffer->data = vertices;
    // vertexBuffer->indices = nullptr;

    auto normalBuffer = vsg::BufferInfo::create();
    normalBuffer->data = normals;
    // normalBuffer->indices = nullptr;

    auto texCoordBuffer = vsg::BufferInfo::create();
    texCoordBuffer->data = texCoords;
    // texCoordBuffer->indices = nullptr;

    // Create index buffer
    auto indexBuffer = vsg::BufferInfo::create();
    indexBuffer->data = nullptr;
    // indexBuffer->indices = indices;

    // Assign arrays to geometry
    geometry->arrays.push_back(vertexBuffer);
    geometry->arrays.push_back(normalBuffer);
    geometry->arrays.push_back(texCoordBuffer);
    geometry->indices = indexBuffer;

    // Add draw command
    geometry->commands.push_back(vsg::DrawIndexed::create(indexData.size(), 1, 0, 0, 0));

    return geometry;
}

vsg::ref_ptr<vsg::Node> Base::makeGround(const VsgConfig& config) {
    float size = 1000.0f;
    float texScale = 0.2f;

    // Create vertex data
    std::vector<vsg::dvec3> vertexData = {
        {-size, -size, 0.0f},
        {size, -size, 0.0f},
        {size, size, 0.0f},
        {-size, size, 0.0f}
    };

    std::vector<vsg::vec2> texCoordData = {
        {-texScale * size, -texScale * size},
        {texScale * size, -texScale * size},
        {texScale * size, texScale * size},
        {-texScale * size, texScale * size}
    };

    std::vector<uint16_t> indexData = {
        0, 1, 2,
        2, 3, 0
    };

    // Create arrays
    auto vertices = vsg::dvec3Array::create(vertexData.size(), vertexData.data());
    auto texCoords = vsg::vec2Array::create(texCoordData.size(), texCoordData.data());
    auto indices = vsg::ushortArray::create(indexData.size(), indexData.data());

    // Create geometry
    auto geometry = vsg::Geometry::create();

    // Create vertex input bindings
    vsg::VertexInputState::Bindings vertexBindings{
        VkVertexInputBindingDescription{0, sizeof(vsg::dvec3), VK_VERTEX_INPUT_RATE_VERTEX},
        VkVertexInputBindingDescription{1, sizeof(vsg::vec2), VK_VERTEX_INPUT_RATE_VERTEX}
    };

    // Create vertex input attributes
    vsg::VertexInputState::Attributes vertexAttributes{
        VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0}, // vertices
        VkVertexInputAttributeDescription{1, 1, VK_FORMAT_R32G32_SFLOAT, 0}     // texcoords
    };

    // Create vertex input state
    auto vertexInputState = vsg::VertexInputState::create(vertexBindings, vertexAttributes);

    // Create vertex buffers
    auto vertexBuffer = vsg::BufferInfo::create();
    vertexBuffer->data = vertices;
    // vertexBuffer->indices = nullptr;

    auto texCoordBuffer = vsg::BufferInfo::create();
    texCoordBuffer->data = texCoords;
    // texCoordBuffer->indices = nullptr;

    // Create index buffer
    auto indexBuffer = vsg::BufferInfo::create();
    indexBuffer->data = nullptr;
    // indexBuffer->indices = indices;

    // Assign arrays to geometry
    geometry->arrays.push_back(vertexBuffer);
    geometry->arrays.push_back(texCoordBuffer);
    geometry->indices = indexBuffer;

    // Add draw command
    geometry->commands.push_back(vsg::DrawIndexed::create(indexData.size(), 1, 0, 0, 0));

    return geometry;
}


void Base::makeLights(vsg::ref_ptr<vsg::Group> node, const VsgConfig& config) {
    // Define light data structure
    struct LightData {
        vsg::vec4 position;
        vsg::vec4 ambient;
        vsg::vec4 diffuse;
        vsg::vec4 specular;
    };

    // Create light data
    auto lightData = vsg::Value<LightData>::create();
    lightData->value().position = vsg::vec4(1.0f, 1.0f, 1.0f, 0.0f); // Directional light
    lightData->value().ambient = vsg::vec4(0.3f, 0.3f, 0.3f, 1.0f);
    lightData->value().diffuse = vsg::vec4(1.0f, 1.0f, 1.0f, 1.0f);
    lightData->value().specular = vsg::vec4(1.0f, 1.0f, 1.0f, 1.0f);

    // Create descriptor buffer
    auto lightDescriptor = vsg::DescriptorBuffer::create(lightData, 0, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);

    // Create binding description
    VkDescriptorSetLayoutBinding layoutBinding{};
    layoutBinding.binding = 0;
    layoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    layoutBinding.descriptorCount = 1;
    layoutBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    layoutBinding.pImmutableSamplers = nullptr;

    // Create descriptor set layout
    auto lightDescriptorSetLayout = vsg::DescriptorSetLayout::create(vsg::DescriptorSetLayoutBindings{layoutBinding});

    // Create descriptor set
    auto lightDescriptorSet = vsg::DescriptorSet::create(lightDescriptorSetLayout, vsg::Descriptors{lightDescriptor});

    // Create pipeline layout (needed for the bind descriptor set)
    auto pipelineLayout = vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{lightDescriptorSetLayout}, vsg::PushConstantRanges{});

    // Create bind descriptor set command
    auto bindLightDescriptorSet = vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 1, lightDescriptorSet);

    // Add the bind command to the node
    node->addChild(bindLightDescriptorSet);
}



vsg::ref_ptr<vsg::Node> Base::createShadowedScene(vsg::ref_ptr<vsg::Node> sceneToShadow,
                                                  vsg::ref_ptr<vsg::Light> lightSource,
                                                  int shadowType) {
    // Implement shadow mapping techniques using shaders and render passes
    // This is an advanced topic and requires setting up depth maps, framebuffers, and appropriate shaders

    // For this example, we'll return the scene without shadows
    return sceneToShadow;
}


vsg::ref_ptr<vsg::Node> Base::createHUD(VsgScene* scene, const VsgConfig& config) {
    return nullptr;
}


void Base::setCaption(const std::string& caption) {
    this->caption = caption;
    // Update HUD if necessary
}

void Base::setTitle(const std::string& title) {
    this->title = title;
    // Update HUD if necessary
}

void Base::setTimeStats(double time, double realtimefactor,
                        double truerealtimefactor, bool pause) {
    // Implement time stats update
}

void Base::changeShadowTechnique() {
    // Implement shadow technique change if applicable
}

int Base::contains(char** list, int len, const char* str) {
    for (int i = 0; i < len; i++) {
        if (strcmp(list[i], str) == 0)
            return i + 1;
    }
    return 0;
}

} // namespace lpzrobots
