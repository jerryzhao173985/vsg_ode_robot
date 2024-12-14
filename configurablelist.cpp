#include "configurablelist.h"

using namespace std;

ConfigurableList::ConfigurableList() {}

ConfigurableList::~ConfigurableList() {
  callBack(CALLBACK_CONFIGURABLE_LIST_BEING_DELETED);
}


void ConfigurableList::push_back(Configurable* const & configurable) {
  vector<Configurable*>::push_back(configurable);
  callBack(CALLBACK_CONFIGURABLE_LIST_MODIFIED);
}


void ConfigurableList::pop_back() {
  vector<Configurable*>::pop_back();
  callBack(CALLBACK_CONFIGURABLE_LIST_MODIFIED);
}


void ConfigurableList::clear() {
  vector<Configurable*>::clear();
  callBack(CALLBACK_CONFIGURABLE_LIST_MODIFIED);
}


ConfigurableList::iterator ConfigurableList::erase(iterator pos){
  ConfigurableList::iterator i  = vector<Configurable*>::erase(pos);
  callBack(CALLBACK_CONFIGURABLE_LIST_MODIFIED);
  return i;  
}
