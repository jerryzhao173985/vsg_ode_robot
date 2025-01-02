#include "joint.h"
#include "pos.h"
#include "odehandle.h"
#include "vsghandle.h"
#include <vsg/all.h>
#include <cmath> // for fabs
#include <algorithm> // for std::find

#include "primitive.h"  // We now use the primitives from the new implementation

namespace lpzrobots {

    // Add these helper functions at the top of the joint.cpp file
    bool isValidVector(const dReal* v, int size=3) {
        if (!v) return false;
        double sumSq = 0;
        for (int i = 0; i < size; i++) {
            if (std::isnan(v[i]) || std::isinf(v[i])) return false;
            sumSq += v[i] * v[i];
        }
        return sumSq > 1e-10;  // Check for near-zero vector
    }

    // Modify anchorAxisPose to include validation:
    vsg::dmat4 Joint::anchorAxisPose(const vsg::dvec3& anchor, const Axis& axis) {
        vsg::dvec3 safeAxis = axis.toVec3();
        
        // Validate axis
        double len = std::sqrt(safeAxis.x*safeAxis.x + safeAxis.y*safeAxis.y + safeAxis.z*safeAxis.z);
        if (len < 1e-10 || std::isnan(len) || std::isinf(len)) {
            std::cerr << "Warning: Invalid axis in anchorAxisPose, using default" << std::endl;
            safeAxis = vsg::dvec3(0.0, 0.0, 1.0);
        } else {
            safeAxis = safeAxis * (1.0/len);  // Normalize
        }
        
        // Create quaternion that rotates from z-axis to target axis
        vsg::dquat rotation(vsg::dvec3(0.0, 0.0, 1.0), safeAxis);
        return vsg::translate(anchor) * vsg::rotate(rotation);
    }


    Joint::~Joint(){
        setFeedBackMode(false);
        if (joint) dJointDestroy(joint);
        if(part1->getGeom() && part2->getGeom()){
            odeHandle.removeIgnoredPair(part1->getGeom(), part2->getGeom());
        }
    }

    void Joint::init(const OdeHandle& odeHandle, const VsgHandle& /*vsgHandle*/,
                     bool /*withVisual*/, double /*visualSize*/, bool ignoreColl){
        this->odeHandle = odeHandle;
        assert(part1->getBody() && "MSG: only part2 can be static");
        if(ignoreColl && part1->getGeom() && part2->getGeom())
            this->odeHandle.addIgnoredPair(part1->getGeom(), part2->getGeom());
    }

    void Joint::setFeedBackMode(bool mode) {
        if (mode){
            if((feedback=dJointGetFeedback(joint))==0){
                feedback = (dJointFeedback*)malloc(sizeof(dJointFeedback));
                dJointSetFeedback (joint, feedback);
            }
        }else{
            if(feedback){
                dJointSetFeedback (joint, 0);
                free(feedback);
                feedback=0;
            }
        }
    }

    bool Joint::getTorqueFeedback(Pos& t1, Pos& t2) const {
        dJointFeedback* fb = dJointGetFeedback(joint);
        if (!fb) return false;
        t1 = Pos(fb->t1);
        t2 = Pos(fb->t2);
        return true;
    }

    bool Joint::getForceFeedback(Pos& f1, Pos& f2) const {
        dJointFeedback* fb = dJointGetFeedback(joint);
        if (!fb) return false;
        f1 = Pos(fb->f1);
        f2 = Pos(fb->f2);
        return true;
    }

    /***************************************************************************/
    // OneAxisJoint
    std::list<double> OneAxisJoint::getPositions() const {
        std::list<double> l;
        l.push_back(getPosition1());
        return l;
    }

    std::list<double> OneAxisJoint::getPositionRates() const {
        std::list<double> l;
        l.push_back(getPosition1Rate());
        return l;
    }

    int OneAxisJoint::getPositions(double* sensorarray) const {
        sensorarray[0] = getPosition1();
        return 1;
    }

    int OneAxisJoint::getPositionRates(double* sensorarray) const{
        sensorarray[0] = getPosition1Rate();
        return 1;
    }

    /***************************************************************************/
    // TwoAxisJoint
    std::list<double> TwoAxisJoint::getPositions() const {
        std::list<double> l;
        l.push_back(getPosition1());
        l.push_back(getPosition2());
        return l;
    }

    std::list<double> TwoAxisJoint::getPositionRates() const {
        std::list<double> l;
        l.push_back(getPosition1Rate());
        l.push_back(getPosition2Rate());
        return l;
    }

    int TwoAxisJoint::getPositions(double* sensorarray) const {
        sensorarray[0] = getPosition1();
        sensorarray[1] = getPosition2();
        return 2;
    }

    int TwoAxisJoint::getPositionRates(double* sensorarray) const{
        sensorarray[0] = getPosition1Rate();
        sensorarray[1] = getPosition2Rate();
        return 2;
    }

    /***************************************************************************/
    // FixedJoint
    FixedJoint::FixedJoint(Primitive* part1, Primitive* part2,
                           const vsg::dvec3& anchor)
    : Joint(part1, part2, anchor), visual(0) {
        // anchor is here used as a relative position to part1
        if(anchor.x!=0 || anchor.y!=0 || anchor.z!=0 ){ 
            this->anchor = part1->toLocal(anchor);
        } 
    }

    FixedJoint::~FixedJoint(){
        if (visual) delete visual;
    }

    void FixedJoint::init(const OdeHandle& odeHandle, const VsgHandle& vsgHandle,
                          bool withVisual, double visualSize, bool ignoreColl){
        Joint::init(odeHandle, vsgHandle, withVisual, visualSize, ignoreColl);
        joint = dJointCreateFixed (odeHandle.world,0);
        dJointAttach (joint, part1->getBody(),part2->getBody());
        dJointSetFixed (joint);

        if(withVisual){
            visual = new Sphere(visualSize/2.0);
            visual->init(odeHandle, 0.0, vsgHandle, Primitive::Geom | Primitive::Draw);
            vsg::dmat4 poseMat = vsg::translate(anchor.x, anchor.y, anchor.z) * part1->getPose().getMatrix();
            visual->setPose(Pose(poseMat));
        }
    }

    void FixedJoint::update(){
        if(visual){
            vsg::dmat4 poseMat = vsg::translate(anchor.x, anchor.y, anchor.z) * part1->getPose().getMatrix();
            visual->setPose(Pose(poseMat));
        }
    }

    void FixedJoint::setParam(int /*parameter*/, double /*value*/) {
    }

    double FixedJoint::getParam(int /*parameter*/) const {
        return 0;
    }

    /***************************************************************************/
    // HingeJoint
    HingeJoint::HingeJoint(Primitive* part1, Primitive* part2, const vsg::dvec3& anchor,
                           const Axis& axis1)
    : OneAxisJoint(part1, part2, anchor, axis1),  visual(0) {
    }

    HingeJoint::~HingeJoint(){
        if (visual) delete visual;
    }

    // Modify joint initialization to validate axes
    void HingeJoint::init(const OdeHandle& odeHandle, const VsgHandle& vsgHandle,
                          bool withVisual, double visualSize, bool ignoreColl) {
        Joint::init(odeHandle, vsgHandle, withVisual, visualSize, ignoreColl);
        
        // Validate axis1
        vsg::dvec3 safeAxis = axis1.toVec3();
        double len = std::sqrt(safeAxis.x*safeAxis.x + safeAxis.y*safeAxis.y + safeAxis.z*safeAxis.z);
        if (len < 1e-10) {
            std::cerr << "Warning: Invalid hinge axis, using default" << std::endl;
            safeAxis = vsg::dvec3(0.0, 0.0, 1.0);
            axis1 = Axis(safeAxis.x, safeAxis.y, safeAxis.z);
        }
        
        joint = dJointCreateHinge(odeHandle.world, 0);
        dJointAttach(joint, part1->getBody(), part2->getBody());
        dJointSetHingeAnchor(joint, anchor.x, anchor.y, anchor.z);
        dJointSetHingeAxis(joint, axis1[0], axis1[1], axis1[2]);
        if(withVisual){
            visual = new Cylinder(visualSize/15.0, visualSize);
            visual->init(odeHandle, 0.0, vsgHandle, Primitive::Geom | Primitive::Draw);
            vsg::dmat4 t = anchorAxisPose(anchor, axis1);
            visual->setPose(Pose(t));
        }
    }

    void HingeJoint::update(){
        if(visual){
            dVector3 v;
            dJointGetHingeAnchor(joint, v);
            anchor = vsg::dvec3(v[0], v[1], v[2]);
            dJointGetHingeAxis(joint, v);
            axis1 = Axis(v[0], v[1], v[2]);
            visual->setPose(Pose(anchorAxisPose(anchor, axis1)));
        }
    }

    void HingeJoint::addForce1(double t){
        dJointAddHingeTorque(joint, t);
    }

    double HingeJoint::getPosition1() const{
        return dJointGetHingeAngle(joint);
    }

    double HingeJoint::getPosition1Rate() const{
        return dJointGetHingeAngleRate(joint);
    }

    void HingeJoint::setParam(int parameter, double value) {
        assert(joint); 
        dJointSetHingeParam(joint, parameter, value);
    }

    double HingeJoint::getParam(int parameter) const{
        assert(joint);
        return dJointGetHingeParam(joint, parameter);
    }

    /***************************************************************************/
    // Hinge2Joint
    Hinge2Joint::Hinge2Joint(Primitive* part1, Primitive* part2, const vsg::dvec3& anchor,
                             const Axis& axis1, const Axis& axis2)
    : TwoAxisJoint(part1, part2, anchor, axis1, axis2), visual(0) {
    }

    Hinge2Joint::~Hinge2Joint(){
        if (visual) delete visual;
    }

    void Hinge2Joint::init(const OdeHandle& odeHandle, const VsgHandle& vsgHandle,
                           bool withVisual, double visualSize, bool ignoreColl){
        Joint::init(odeHandle, vsgHandle, withVisual, visualSize, ignoreColl);
        joint = dJointCreateHinge2 (odeHandle.world,0);
        dJointAttach (joint, part1->getBody(),part2->getBody());
        dJointSetHinge2Anchor (joint, anchor.x, anchor.y, anchor.z);
        dJointSetHinge2Axis1 (joint, axis1[0], axis1[1], axis1[2]);
        dJointSetHinge2Axis2 (joint, axis2[0], axis2[1], axis2[2]);
        if(withVisual){
            visual = new Cylinder(visualSize/15.0, visualSize);
            visual->init(odeHandle, 0.0, vsgHandle, Primitive::Geom | Primitive::Draw);
            vsg::dmat4 t = anchorAxisPose(anchor, axis2);
            visual->setPose(Pose(t));
        }
    }

    void Hinge2Joint::update(){
        if(visual){
            dVector3 v;
            dJointGetHinge2Anchor(joint, v);
            anchor = vsg::dvec3(v[0], v[1], v[2]);
            dJointGetHinge2Axis2(joint, v);
            axis2 = Axis(v[0], v[1], v[2]);
            visual->setPose(Pose(anchorAxisPose(anchor, axis2)));
        }
    }

    void Hinge2Joint::addForce1(double t1){
        dJointAddHinge2Torques(joint, t1, 0);
    }

    void Hinge2Joint::addForce2(double t2){
        dJointAddHinge2Torques(joint, 0, t2);
    }

    double Hinge2Joint::getPosition1()  const{
        return dJointGetHinge2Angle1(joint);
    }

    double Hinge2Joint::getPosition2() const{
        // Not supported by ODE hinge2
        fprintf(stderr, "Hinge2Joint::getPosition2() is called, but not supported!\n");
        return 0;
    }

    double Hinge2Joint::getPosition1Rate() const{
        return dJointGetHinge2Angle1Rate(joint);
    }

    double Hinge2Joint::getPosition2Rate() const{
        return dJointGetHinge2Angle2Rate(joint);
    }

    void Hinge2Joint::setParam(int parameter, double value) {
        assert(joint);
        dJointSetHinge2Param(joint, parameter, value);
    }

    double Hinge2Joint::getParam(int parameter) const{
        assert(joint);
        return dJointGetHinge2Param(joint, parameter);
    }

    /***************************************************************************/
    // UniversalJoint
    UniversalJoint::UniversalJoint(Primitive* part1, Primitive* part2, const vsg::dvec3& anchor,
                                   const Axis& axis1, const Axis& axis2)
    : TwoAxisJoint(part1, part2, anchor, axis1, axis2), visual1(0), visual2(0) {
    }

    UniversalJoint::~UniversalJoint(){
        if (visual1) delete visual1;
        if (visual2) delete visual2;
    }

    void UniversalJoint::init(const OdeHandle& odeHandle, const VsgHandle& vsgHandle,
                              bool withVisual, double visualSize, bool ignoreColl){
        Joint::init(odeHandle, vsgHandle, withVisual, visualSize, ignoreColl);
        joint = dJointCreateUniversal (odeHandle.world,0);
        dJointAttach (joint, part1->getBody(),part2->getBody());
        dJointSetUniversalAnchor (joint, anchor.x, anchor.y, anchor.z);
        dJointSetUniversalAxis1 (joint, axis1[0], axis1[1], axis1[2]);
        dJointSetUniversalAxis2 (joint, axis2[0], axis2[1], axis2[2]);
        if(withVisual){
            visual1 = new Cylinder(visualSize/15.0, visualSize);
            visual1->init(odeHandle, 0.0, vsgHandle, Primitive::Geom | Primitive::Draw);
            vsg::dmat4 t = anchorAxisPose(anchor, axis1);
            visual1->setPose(Pose(t));

            visual2 = new Cylinder(visualSize/15.0, visualSize);
            visual2->init(odeHandle, 0.0, vsgHandle, Primitive::Geom | Primitive::Draw);
            t = anchorAxisPose(anchor, axis2);
            visual2->setPose(Pose(t));
        }
    }

    // Add validation to joint updates
    void UniversalJoint::update() {
        if(visual1 && visual2) {
            dVector3 v;
            dJointGetUniversalAnchor(joint, v);
            anchor = vsg::dvec3(v[0], v[1], v[2]);
            
            dJointGetUniversalAxis1(joint, v);
            if (isValidVector(v)) {
                axis1 = Axis(v[0], v[1], v[2]);
            } else {
                std::cerr << "Warning: Invalid universal joint axis1" << std::endl;
                return;  // Skip update if axes are invalid
            }
            
            dJointGetUniversalAxis2(joint, v);
            if (isValidVector(v)) {
                axis2 = Axis(v[0], v[1], v[2]);
            } else {
                std::cerr << "Warning: Invalid universal joint axis2" << std::endl;
                return;
            }
            
            visual1->setPose(Pose(anchorAxisPose(anchor, axis1)));
            visual2->setPose(Pose(anchorAxisPose(anchor, axis2)));
        }
    }

    void UniversalJoint::addForce1(double t1){
        dJointAddUniversalTorques(joint, t1,0);
    }

    void UniversalJoint::addForce2(double t2){
        dJointAddUniversalTorques(joint, 0,t2);
    }

    double UniversalJoint::getPosition1() const{
        return dJointGetUniversalAngle1(joint);
    }

    double UniversalJoint::getPosition2() const{
        return dJointGetUniversalAngle2(joint);
    }

    double UniversalJoint::getPosition1Rate() const{
        return dJointGetUniversalAngle1Rate(joint);
    }

    double UniversalJoint::getPosition2Rate() const{
        return dJointGetUniversalAngle2Rate(joint);
    }

    void UniversalJoint::setParam(int parameter, double value) {
        dJointSetUniversalParam(joint, parameter, value);
    }

    double UniversalJoint::getParam(int parameter) const{
        return dJointGetUniversalParam(joint, parameter);
    }

    /***************************************************************************/
    // BallJoint
    BallJoint::BallJoint(Primitive* part1, Primitive* part2, const vsg::dvec3& anchor)
    : Joint(part1, part2, anchor), visual(0) {
    }

    BallJoint::~BallJoint() {
        if (visual) delete visual;
    }

    void BallJoint::init(const OdeHandle& odeHandle, const VsgHandle& vsgHandle,
                         bool withVisual, double visualSize, bool ignoreColl){
        Joint::init(odeHandle, vsgHandle, withVisual, visualSize, ignoreColl);
        joint = dJointCreateBall(odeHandle.world, 0);
        dJointAttach (joint, part1->getBody(),part2->getBody());
        dJointSetBallAnchor (joint, anchor.x, anchor.y, anchor.z);
        if(withVisual){
            visual = new Sphere(visualSize);
            visual->init(odeHandle, 0.0, vsgHandle, Primitive::Geom | Primitive::Draw);
            vsg::dmat4 t = vsg::translate(anchor.x, anchor.y, anchor.z);
            // Anchor is in global coords at first, but we can just set pose directly:
            // Actually anchor was relative, but after init, anchor is global. 
            // part1 might move, so if needed we can get an updated anchor from ODE
            visual->setPose(Pose(t));
        }
    }

    void BallJoint::update(){
        if(visual){
            dVector3 v;
            dJointGetBallAnchor(joint, v);
            anchor = vsg::dvec3(v[0], v[1], v[2]);
            vsg::dmat4 t = vsg::translate(anchor.x, anchor.y, anchor.z);
            visual->setPose(Pose(t));
        }
    }

    void BallJoint::setParam(int /*parameter*/, double /*value*/) { }

    double BallJoint::getParam(int /*parameter*/) const{
        return 0;
    }

    /***************************************************************************/
    // SliderJoint
    SliderJoint::SliderJoint(Primitive* part1, Primitive* part2, const vsg::dvec3& anchor,
                             const Axis& axis1)
    : OneAxisJoint(part1, part2, anchor, axis1), visual(0), visualSize(0) {
    }

    SliderJoint::~SliderJoint(){
        if (visual) delete visual;
    }

    void SliderJoint::init(const OdeHandle& odeHandle, const VsgHandle& vsgHandle,
                           bool withVisual, double visualSize, bool ignoreColl){
        this->vsgHandle= vsgHandle;
        this->visualSize = visualSize;
        Joint::init(odeHandle, vsgHandle, withVisual, visualSize, ignoreColl);

        joint = dJointCreateSlider (odeHandle.world,0);
        dJointAttach (joint, part1->getBody(),part2->getBody());
        dJointSetSliderAxis (joint, axis1[0], axis1[1], axis1[2]);
        if(withVisual){
            double len = getPosition1();
            visual = new Cylinder(visualSize/10.0, std::max(len+visualSize,0.001));
            visual->init(odeHandle, 0.0, vsgHandle, Primitive::Geom | Primitive::Draw);
            vsg::dmat4 t = anchorAxisPose(anchor, axis1);
            visual->setPose(Pose(t));
        }
    }

    void SliderJoint::update(){
        if(visual){
            vsg::dvec3 p1 = part1->getPosition();
            vsg::dvec3 p2 = part2->getPosition();
            anchor = (p1+p2)*0.5;
            dVector3 v;
            dJointGetSliderAxis(joint, v);
            axis1 = Axis(v[0], v[1], v[2]);

            double len = getPosition1();

            // Remove old visual from scene graph before deleting
            if (visual->getTransformNode() && vsgHandle.parent) {
                auto& children = vsgHandle.parent->children;
                auto it = std::find(children.begin(), children.end(), visual->getTransformNode());
                if (it != children.end()) children.erase(it);
            }

            delete visual;
            visual = new Cylinder(visualSize/10.0, std::max(len+visualSize,0.001));
            visual->init(odeHandle, 0.0, vsgHandle, Primitive::Geom | Primitive::Draw);
            vsg::dmat4 t = anchorAxisPose(anchor, axis1);
            visual->setPose(Pose(t));
        }
    }

    void SliderJoint::addForce1(double t){
        dJointAddSliderForce(joint, t);
    }

    double SliderJoint::getPosition1() const{
        return dJointGetSliderPosition(joint);
    }

    double SliderJoint::getPosition1Rate() const{
        return dJointGetSliderPositionRate(joint);
    }

    void SliderJoint::setParam(int parameter, double value) {
        assert(joint);
        dJointSetSliderParam(joint, parameter, value);
    }

    double SliderJoint::getParam(int parameter) const{
        assert(joint);
        return dJointGetSliderParam(joint, parameter);
    }

} // namespace lpzrobots