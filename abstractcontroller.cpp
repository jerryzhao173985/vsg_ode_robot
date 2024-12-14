#include "abstractcontroller.h"

using namespace std;

void AbstractController::sensorInfos(std::list<SensorMotorInfo> sensorInfos) {
  FOREACHIa(sensorInfos, s, i){
    sensorIndexMap[s->name] = i;
    sensorInfoMap[i] = *s;
  }
};

void AbstractController::motorInfos(std::list<SensorMotorInfo> motorInfos) {
  FOREACHIa(motorInfos, m, i){
    motorIndexMap[m->name] = i;
    motorInfoMap[i] = *m;
  }
};

int AbstractController::SIdx(const std::string& name){
  auto it = sensorIndexMap.find(name);
  if (it!=sensorIndexMap.end()){
    return it->second;
  }else{
    cerr << "Cannot find Sensor with name \"" << name << "\""
         << " Possible values are:" << endl;
    for(auto& i: sensorIndexMap){
      cerr << i.first << ", ";
    }
    cerr << endl;
    return 0;
  }
}

int AbstractController::MIdx(const std::string& name){
  auto it = motorIndexMap.find(name);
  if (it!=motorIndexMap.end()){
    return it->second;
  }else{
    cerr << "Cannot find Motor with name \"" << name << "\""
         << " Possible values are:" << endl;
    for(auto& i: motorIndexMap){
      cerr << i.first << ", ";
    }
    cerr << endl;
    return 0;
  }
}

SensorMotorInfo AbstractController::SInfo(int index){
  auto it = sensorInfoMap.find(index);
  if (it!=sensorInfoMap.end()){
    return it->second;
  }else{
    cerr << "No info for Sensor with index " << index << "! Out of bounds?"<< endl;
    return SensorMotorInfo();
  }}

SensorMotorInfo AbstractController::MInfo(int index){
  auto it = motorInfoMap.find(index);
  if (it!=motorInfoMap.end()){
    return it->second;
  }else{
    cerr << "No info for Motor with index " << index << "! Out of bounds?"<< endl;
    return SensorMotorInfo();
  }
}
