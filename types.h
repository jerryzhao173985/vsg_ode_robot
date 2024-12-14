#ifndef __UTIL_TYPES_H
#define __UTIL_TYPES_H

#define CHANGER(obj, vartype, var) obj change##var(vartype _##var) { obj s = *this; s.var=_##var; return s; }

typedef double sensor;
typedef double motor;

#endif
