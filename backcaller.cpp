#include "backcaller.h"
#include "callbackable.h"
#include <algorithm>
#include "stl_adds.h"
#include "quickmp.h"

BackCaller::BackCaller() {}

BackCaller::~BackCaller()
{
  // remove all lists in callbackableMap
  FOREACH(callbackableMapType, callbackableMap, mapItr)
  {
    delete (*mapItr).second;
  }
  callbackableMap.clear();
}

void BackCaller::addCallbackable(Callbackable *callbackableInstance, CallbackableType type /* = BackCaller::DEFAULT_CALLBACKABLE_TYPE*/)
{
  callbackableListType* list = 0;
  callbackableMapType::const_iterator iterator = callbackableMap.find(type);
  if (iterator!= callbackableMap.end())
    list = (*iterator).second;
  else
  { // not found, create new one
    list = new callbackableListType();
    callbackableMap[type] = list;
  }
  // check if callbackableInstance is already registered
  callbackableListType::const_iterator listItr = find(list->begin(), list->end(), callbackableInstance);
  if (listItr==list->end())
    list->push_back(callbackableInstance);
}


void BackCaller::removeCallbackable(Callbackable *callbackableInstance, CallbackableType type /* = BackCaller::DEFAULT_CALLBACKABLE_TYPE*/)
{
  callbackableListType* list = 0;
  callbackableMapType::const_iterator iterator = callbackableMap.find(type);
  if (iterator!= callbackableMap.end())
    list = (*iterator).second;
  else // not found, ignore
    return;
  callbackableListType::iterator listItr = find(list->begin(), list->end(), callbackableInstance);
  if (listItr!=list->end())
    list->erase(listItr);
}

void BackCaller::removeAllCallbackables(CallbackableType type /* = BackCaller::DEFAULT_CALLBACKABLE_TYPE */)
{
  callbackableListType* list = 0;
  callbackableMapType::const_iterator iterator = callbackableMap.find(type);
  if (iterator!= callbackableMap.end())
    list = (*iterator).second;
  else // not found, ignore
    return;
  list->clear();
}


void BackCaller::callBack(CallbackableType type /* = BackCaller::DEFAULT_CALLBACKABLE_TYPE*/)
{
  callbackableListType* list = 0;
  callbackableMapType::const_iterator iterator = callbackableMap.find(type);
  if (iterator!= callbackableMap.end())
    list = (*iterator).second;
  else // not found, ignore
    return;
  FOREACHC(callbackableListType, *list, listItr)
  {
    (*listItr)->doOnCallBack(this, type);
  }
}


void BackCaller::callBackQMP(CallbackableType type /* = BackCaller::DEFAULT_CALLBACKABLE_TYPE*/)
{
  callbackableListType* list = 0;
  callbackableMapType::const_iterator iterator = callbackableMap.find(type);
  if (iterator!= callbackableMap.end())
    list = (*iterator).second;
  else // not found, ignore
    return;
  unsigned int listSize = list->size();
  if(listSize==1){
    list->front()->doOnCallBack(this, type);
  }else if (listSize>1){
    callbackableListType derefList = *list; // use dereferenced version to avoid parallel accesses to the pointer (but i guess that the compiler optimises it already)
    BackCaller thisCaller = *this;
    QMP_SHARE(derefList);
    QMP_SHARE(type);
    QMP_SHARE(thisCaller);
    QMP_PARALLEL_FOR(i, 0, listSize){
      QMP_USE_SHARED(derefList, callbackableListType);
      QMP_USE_SHARED(type, CallbackableType);
      QMP_USE_SHARED(thisCaller, BackCaller);
      derefList[i]->doOnCallBack(&thisCaller, type);
    }
    QMP_END_PARALLEL_FOR;
  }
}

