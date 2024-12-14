#ifndef __CONFIGURABLELIST_H_
#define __CONFIGURABLELIST_H_

#include <vector>
#include "configurable.h"

/**
 * Establishes for some methods the notifications for registered Callbackable instances
 * (use addCallbackable(...)).
 * @warning Only the following methods are currently supported: 
    push_back(...), pop_back(), erase() and clear()!
 * You can use iterators with the limitation to not delete or insert.
 */
class ConfigurableList : public std::vector<Configurable*>, public BackCaller {
  public:
    ConfigurableList();
    virtual ~ConfigurableList();

    /**
     * Indicates that the list has been modified, a Configurable instance was either added or removed.
     */
    static const CallbackableType CALLBACK_CONFIGURABLE_LIST_MODIFIED = 3;

    /**
     * Indicates that the list is being deleted.
     */
    static const CallbackableType CALLBACK_CONFIGURABLE_LIST_BEING_DELETED = 4;

    void push_back(Configurable* const & configurable);
    iterator erase(iterator pos);
    void pop_back();
    void clear();
};

#endif /* __CONFIGURABLELIST_H_ */
