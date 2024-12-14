#include "odehandle.h"
#include <assert.h>
#include <ode/ode.h>
#include <algorithm>
#include "primitive.h"

namespace lpzrobots
{

  OdeHandle::OdeHandle()
  {
    ignoredPairs        = 0;
    spaces              = 0;
  }

  OdeHandle::OdeHandle(  dWorldID _world, dSpaceID _space, dJointGroupID _jointGroup )
  {
    world               = _world;
    space               = _space;
    jointGroup          = _jointGroup;
    ignoredPairs        = 0;
    spaces              = 0;
  }

  void OdeHandle::destroySpaces()
  {
    if (spaces)
      delete spaces;

    if (ignoredPairs)
      delete ignoredPairs;
  }

  void OdeHandle::init(double* time)
  {
    assert(time);
    this->time=time;
    dInitODE();
    world = dWorldCreate ();

    // Create the primary world-space, which is used for collision detection
    space = dHashSpaceCreate (0);
    dSpaceSetCleanup (space, 0);
    spaces = new std::vector<dSpaceID>();
    // the jointGroup is used for collision handling,
    //  where a lot of joints are created every step
    jointGroup = dJointGroupCreate ( 1000000 );
    ignoredPairs  = new HashSet<std::pair<long,long>,geomPairHash >();

  }

  void OdeHandle::close(){
    dJointGroupDestroy  ( jointGroup );
    dWorldDestroy       ( world );
    dSpaceDestroy       ( space );
    destroySpaces();
    dCloseODE();
  }


  void OdeHandle::createNewSimpleSpace(dSpaceID parentspace, bool ignore_inside_collisions){
    space = dSimpleSpaceCreate (parentspace);
    dSpaceSetCleanup (space, 0);
    if(!ignore_inside_collisions)
      addSpace(space);
  }

  void OdeHandle::createNewHashSpace(dSpaceID parentspace, bool ignore_inside_collisions){
    space = dHashSpaceCreate (parentspace);
    dSpaceSetCleanup (space, 0);
    if(!ignore_inside_collisions)
      addSpace(space);
  }

  void OdeHandle::deleteSpace(){
    removeSpace(space);
    dSpaceDestroy(space);
  }


  // adds a space to the list of spaces for collision detection (ignored spaces do not need to be insered)
  void OdeHandle::addSpace(dSpaceID g)
  {
    if(spaces)
      spaces->push_back(g);
  }

  // removes a space from the list of ignored spaces for collision detection
  void OdeHandle::removeSpace(dSpaceID g)
  {
    if(!spaces) return;

    std::vector<dSpaceID>::iterator i = std::find(spaces->begin(), spaces->end(),g);
    if(i!=spaces->end()){
      spaces->erase(i);
    }
  }

  // returns list of all spaces
  const std::vector<dSpaceID>& OdeHandle::getSpaces()
  {
    return *spaces;
  }


  // adds a pair of geoms to the list of ignored geom pairs for collision detection
  void OdeHandle::addIgnoredPair(dGeomID g1, dGeomID g2)
  {
    if (!ignoredPairs) return;

    ignoredPairs->insert(std::pair<long, long>((long)g1,(long)g2));
    ignoredPairs->insert(std::pair<long, long>((long)g2,(long)g1));
  }
  // removes pair of geoms from the list of ignored geom pairs for collision detection
  void OdeHandle::removeIgnoredPair(dGeomID g1, dGeomID g2)
  {
    if (!ignoredPairs)  return;
    if(isIgnoredPair(g1,g2)){
      ignoredPairs->erase(std::pair<long, long>((long)g1,(long)g2));
      ignoredPairs->erase(std::pair<long, long>((long)g2,(long)g1));
    }
  }
  // adds a pair of Primitives to the list of ignored geom pairs for collision detection
  void OdeHandle::addIgnoredPair(Primitive* p1, Primitive* p2)
  {
    if (!ignoredPairs)  return;
    if(!p1->getGeom() || !p2->getGeom()) return;
    addIgnoredPair(p1->getGeom(), p2->getGeom());
  }
  // removes pair of geoms from the list of ignored geom pairs for collision detection
  void OdeHandle::removeIgnoredPair(Primitive* p1, Primitive* p2)
  {
    if (!ignoredPairs) return;
    if(!p1->getGeom() || !p2->getGeom()) return;
    removeIgnoredPair(p1->getGeom(),p2->getGeom());
  }

}