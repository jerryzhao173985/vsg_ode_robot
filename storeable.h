#ifndef __STOREABLE_H
#define __STOREABLE_H

#include<stdio.h>

/**
 * Interface for objects, that can be stored and restored to/from a file stream (binary).
*/

class Storeable {
public:
  virtual ~Storeable(){}
  /** stores the object to the given file stream (ASCII preferred).
  */
  virtual bool store(FILE* f) const = 0;

  /** loads the object from the given file stream (ASCII preferred).
  */
  virtual bool restore(FILE* f) = 0;

  /** Provided for convenience.
      Stores the object into a new file with the given filename
   */
  bool storeToFile(const char* filename) const;

  /** Provided for convenience.
      restores the object from the file given by filename
   */
  bool restoreFromFile(const char* filename);

};

#endif