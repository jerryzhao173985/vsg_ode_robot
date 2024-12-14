#include "globaldata.h"
#include <algorithm>
#include "odeagent.h"
#include "sound.h"

namespace lpzrobots {

  void GlobalData::addTmpObject(TmpObject* i, double duration){
    if(i){
      i->setExpireTime(time+duration);
      uninitializedTmpObjects.push_back(std::pair<double, TmpObject*>(time+duration,i));
    }
  }

  void GlobalData::initializeTmpObjects(const OdeHandle& odeHandle,
                                        const VsgHandle& vsgHandle){
    if(!uninitializedTmpObjects.empty()){
      FOREACH(TmpObjectList, uninitializedTmpObjects, i){
        i->second->init(odeHandle, vsgHandle);
        tmpObjects.insert(TmpObjectList::value_type(i->first, i->second));
      }
      uninitializedTmpObjects.clear();
    }
  }

  void GlobalData::updateTmpObjects(const VsgHandle& vsgHandle){
    if(!tmpObjects.empty()){
      FOREACH(TmpObjectMap, tmpObjects, i){
        i->second->update();
      }
    }
  }

  /// removes a particular temporary display item even if it is not yet expired
  bool GlobalData::removeTmpObject(TmpObject* obj){
    if(!tmpObjects.empty()){
      TmpObjectMap::iterator i = tmpObjects.begin();
      while(i != tmpObjects.end()){
        if( i->second == obj ){
          i->second->deleteObject();
          delete i->second;
          tmpObjects.erase(i);
          return true;
        }
        ++i;
      }
    }
    return false;
  }

  void GlobalData::removeExpiredObjects(double time){
    if(!tmpObjects.empty()){
      if(time<0) time=this->time;
      TmpObjectMap::iterator i = tmpObjects.begin();
      while(i != tmpObjects.end()){
        if( i->first < time ){
          TmpObjectMap::iterator tmp = i;
          ++tmp;
          i->second->deleteObject();
          delete i->second;
          tmpObjects.erase(i);
          i=tmp;
        }else{
          break; // since they are ordered we can stop here
        }
      }
    }

    // remove old signals from sound list
    if(!sounds.empty())
      sounds.remove_if(Sound::older_than(time));
  }


  AgentList& GlobalData::getAgents() {
    transform(agents.begin(), agents.end(), std::back_inserter(baseAgents), dynamic_agent_caster<OdeAgent*> ());
    return baseAgents;
  }

}