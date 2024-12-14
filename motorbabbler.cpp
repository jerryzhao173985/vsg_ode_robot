#include <stdio.h>
#include <cmath>
#include <assert.h>
#include "motorbabbler.h"
#include "controller_misc.h"

using namespace std;
using namespace matrix;

MotorBabbler::MotorBabbler(function func)
  : AbstractController("motorbabbler", "0.2"), randGen(0) {
  addParameterDef("minperiod",  &minPeriod, 50,  1, 1000, "minimal period for babbling (steps)");
  addParameterDef("maxperiod",  &maxPeriod, 250, 2, 2000, "maximal period for babbling (steps)");
  addParameterDef("amplitude",  &amplitude,   1, 0, 1,    "amplitude of motion");
  addParameterDef("resampling", &resampling, 500,1, 2000, "resample frequencies every so often (steps)");
  addParameterDef("mask",       &mask,       ~0, 0, ~0,   "binary mask for selecting motors");

  switch(func){
  case Sine:
    osci=sine; break;
  case SawTooth:
    osci=sawtooth; break;
  default:
    assert("Unknown function type");
  }
  t=0;
  number_sensors=0;
  number_motors=0;
};

void MotorBabbler::init(int sensornumber, int motornumber, RandGen* randGen){
  number_sensors=sensornumber;
  number_motors=motornumber;
  phases.set(motornumber,1);
  frequencies.set(motornumber,1);
  if(randGen)
    this->randGen=randGen;
  else
    this->randGen = new RandGen();
  sampleFrequencies();
};

void MotorBabbler::sampleFrequencies(){
  // period from 50 to 250
  double mean=(maxPeriod+minPeriod)/2;
  double spread = (maxPeriod-minPeriod)/2;
  frequencies.add(frequencies.mapP(randGen, random_minusone_to_one)*spread,mean);
  frequencies.toMap(one_over);

}

void MotorBabbler::stepNoLearning(const sensor* sensors, int number_sensors,
                                  motor* motors, int number_motors) {
  if(t>=resampling){
    sampleFrequencies();
    t=0;
  }
  for (int i=0; i<number_motors; i++){
    if(i>=32 || (mask & (1<<i))){
      if(phases.val(i,0) > 2*M_PI) phases.val(i,0) -= 2*M_PI;
      motors[i]=amplitude*osci(phases.val(i,0));
    }else{
      motors[i]=0;
    }
  }

  phases += frequencies*(2*M_PI);
  t++;
};


double MotorBabbler::sine(double x){
  return sin(x);
}

double MotorBabbler::sawtooth(double x){
  while(x>M_PI) x-=2*M_PI;
  while(x<-M_PI) x+=2*M_PI;
  // x is centered around -PI and PI.
  if(x>-M_PI/2 && x <= M_PI/2)
    return x/M_PI;
  else{
    if(x<0)
      return -(x+M_PI)/M_PI;
    else
      return -(x-M_PI)/M_PI;
  }
}
