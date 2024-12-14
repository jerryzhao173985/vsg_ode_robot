#ifndef _CALLBACKABLE_H
#define _CALLBACKABLE_H

#include "backcaller.h"

/**
 * Interface class for a class which wants to be callback on a certain action.
 * In lpzRobots this should be the most case when the time loop is going to the
 * next step.
 *
 * NEW since 20090731:
 * Use the class BackCaller to get already implemented functions like addCallbackable(...).
 *
 * @see BackCaller
 */
class Callbackable
{
  public:

    Callbackable() {};

    virtual ~Callbackable() {}

    /**
     * This method is invoked when a callback is done from the class where this
     * class is for callback registered
     * @param source the caller instance which did the callback.
     * @param type this type can be used to differ from varying types of callback.
     * @see BackCaller
     */
    virtual void doOnCallBack(BackCaller* source, BackCaller::CallbackableType type = BackCaller::DEFAULT_CALLBACKABLE_TYPE) = 0;


};

#endif