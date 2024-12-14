#include <assert.h>
#include <ode/ode.h>
#include "matrix.h"

#include "raysensorbank.h"

namespace lpzrobots {

  RaySensorBank::RaySensorBank() : initialized(false)
  {  };

  RaySensorBank::~RaySensorBank()
  {
    clear();

    // if (initialized)
      //      this->odeHandle.deleteSpace(); this is automatically done if the parent space is deleted

    initialized = false;
  };

  void RaySensorBank::setInitData(const OdeHandle& odeHandle,
                                  const VsgHandle& vsgHandle,
                                  const vsg::dmat4& pose) {
    PhysicalSensor::setInitData(odeHandle, vsgHandle, pose);
    this->odeHandle.createNewSimpleSpace(odeHandle.space, true);
  }


  void RaySensorBank::init(Primitive* own, Joint* joint ) {
    initialized=true;
  };

  unsigned int RaySensorBank::registerSensor(RaySensor* raysensor, Primitive* body,
                                             const vsg::dmat4& pose, float range,
                                             RaySensor::rayDrawMode drawMode){
    assert(isInitDataSet);
    raysensor->setDrawMode(drawMode);
    raysensor->setRange(range);
    raysensor->setInitData(odeHandle, vsgHandle, pose);
    raysensor->init(body);
    bank.push_back(raysensor);
    return bank.size();
  };

  bool RaySensorBank::sense(const GlobalData& global){
    for (unsigned int i=0; i<bank.size(); i++) {
        bank[i]->sense(global);
    }
    return true;
  };

  int RaySensorBank::get(double* sensorarray, int array_size) const {
    int counter=0;
    for(int i=0; (i<array_size) && (i<(int)bank.size()); i++){
      bank[i]->get(&sensorarray[i], 1);
      counter++;
    }
    return counter;
  };

  std::list<sensor> RaySensorBank::getList() const {
    return getListOfArray();
  }

  int RaySensorBank::getSensorNumber() const {
    return bank.size();
  }

  void RaySensorBank::setRange(unsigned int index, float range){
    assert(index<bank.size());
    return bank[index]->setRange(range);
  }

  void RaySensorBank::setRange(float range){
    for(unsigned int i=0; i<bank.size(); i++){
      bank[i]->setRange(range);
    }
  }


  dSpaceID RaySensorBank::getSpaceID(){
    return odeHandle.space;
  };

  void RaySensorBank::update(){
    for (unsigned int i=0; i<bank.size(); i++){
      bank[i]->update();
    }
  };

  // delete all registered sensors.
  void RaySensorBank::clear(){
    for (unsigned int i=0; i<bank.size(); i++)
    {
      if(bank[i])
        delete bank[i];
    }
    bank.clear();
    //odeHandle.deleteSpace();
  }


}





