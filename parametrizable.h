#ifndef __PARAMETRIZABLE_H
#define __PARAMETRIZABLE_H

#include "matrix.h"
#include <list>
#include <functional>

typedef std::list<matrix::Matrix> ParameterList;
/// using ParameterList = std::list<matrix::Matrix>;

/**
   Interface for parametrizable controller.
   Which expose a set of parameters that be set from outside.
*/
class Parametrizable {
public:

  virtual ~Parametrizable() {}

  /** Returns a list of matrices that parametrize the controller
   */
  virtual ParameterList getParameters() const = 0;

  /** sets the parameters.
      The list must have the same length as returned by getParameters()
      The matrix dimensions must fit those given by getParameters()
      @return 0 for failure, 1 for success and 2 if parameters had been changed during setting (to be valid)
         such that a get is required to get the actual values
   */
  virtual int setParameters(const ParameterList& params) = 0;;

};

ParameterList lift2PL(const ParameterList& pl1,const ParameterList& pl2,
                      std::function<matrix::Matrix (const matrix::Matrix&, const matrix::Matrix&)> fun);
ParameterList liftPL(const ParameterList& pl,std::function<matrix::Matrix (const matrix::Matrix&)> fun);

ParameterList mapPL(const ParameterList& pl, double (*fun)(double));
ParameterList divCompPL(const ParameterList& pl1,const ParameterList& pl2);

// assign values of pls to pld (keeps references to pld valid)
void assignPL(ParameterList& pld, const ParameterList& pls);

ParameterList addPL(const ParameterList& pl1,const ParameterList& pl2);
ParameterList subtractPL(const ParameterList& pl1,const ParameterList& pl2);
ParameterList scalePL(const ParameterList& pl,double f);

#endif
