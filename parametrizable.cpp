#include "parametrizable.h"
#include "stl_adds.h"
#include <functional>

ParameterList lift2PL(const ParameterList& pl1,const ParameterList& pl2,
                      std::function<matrix::Matrix (const matrix::Matrix&, const matrix::Matrix&)> fun){
  ParameterList res;
  assert(pl1.size()==pl2.size());
  FOREACH2(pl1, pl2, p1, p2){
    res.push_back(fun(*p1,*p2));
  }
  return res;
}

ParameterList liftPL(const ParameterList& pl,std::function<matrix::Matrix (const matrix::Matrix&)> fun){
  ParameterList res;
  for (auto& p : pl)
    res.push_back(fun(p));
  return res;
}

ParameterList mapPL(const ParameterList& pl, double (*fun)(double)){
  return liftPL(pl, [&fun](const matrix::Matrix& m) { return m.map(fun);});
}

// compoment-wise division
ParameterList divCompPL(const ParameterList& pl1,const ParameterList& pl2){
  auto div = [](double d1, double d2) {return d1/d2;};
  return lift2PL(pl1,pl2, [&div](const matrix::Matrix& m1, const matrix::Matrix& m2)
                 {return matrix::Matrix::map2(div, m1,m2);} );
}

void assignPL(ParameterList& pld, const ParameterList& pls){
  ParameterList::const_iterator ps = pls.begin();
  ParameterList::const_iterator __end2=pls.end();
  for( ParameterList::iterator pd = pld.begin(), __end1=pld.end(); pd!= __end1 && ps!= __end2; pd++ , ps++)
    *pd = *ps;
}

ParameterList addPL(const ParameterList& pl1,const ParameterList& pl2){
  return lift2PL(pl1,pl2, [](const matrix::Matrix& m1, const matrix::Matrix& m2) { return m1 + m2; });
}

ParameterList subtractPL(const ParameterList& pl1,const ParameterList& pl2){
  return lift2PL(pl1,pl2, [](const matrix::Matrix& m1, const matrix::Matrix& m2) { return m1 - m2; });
}

ParameterList scalePL(const ParameterList& pl,double f){
  return liftPL(pl, [f](const matrix::Matrix& m) { return m * f; });
}
