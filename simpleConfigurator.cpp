// SimpleConfigurator.cpp
#include "simpleConfigurator.h"
#include <iomanip>
#include <fstream>
#include <sstream>

namespace lpzrobots {

SimpleConfigurator::SimpleConfigurator(ConfigurableList& configList) 
    : configList(configList), running(false) {
}

SimpleConfigurator::~SimpleConfigurator() {
    close();
}

void SimpleConfigurator::displayParameterDetails(Configurable* conf) {
    if (!conf) return;
    
    std::cout << "\nParameters for " << conf->getName() << ":\n";
    std::cout << std::string(40, '-') << "\n";
    
    // Get all parameter names
    std::list<Configurable::paramkey> params = conf->getAllParamNames();
    
    if (params.empty()) {
        std::cout << "No parameters available.\n";
        return;
    }
    
    // Display each parameter with its details
    for (const auto& param : params) {
        double value = conf->getParam(param);
        bool hasIntBounds = conf->hasParamintBounds(param);
        bool hasValBounds = conf->hasParamvalBounds(param);
        
        std::cout << "Parameter: " << param << "\n";
        std::cout << "  Current Value: " << value << "\n";
        
        // Handle bounds based on parameter type
        if (hasValBounds) {
            Configurable::paramvalBounds bounds = conf->getParamvalBounds(param);
            if (bounds.first != bounds.second) {
                std::cout << "  Valid Range: [" << bounds.first << " to " 
                         << bounds.second << "] (double)\n";
            }
        } else if (hasIntBounds) {
            Configurable::paramintBounds bounds = conf->getParamintBounds(param);
            if (bounds.first != bounds.second) {
                std::cout << "  Valid Range: [" << bounds.first << " to " 
                         << bounds.second << "] (int)\n";
            }
        }
        
        std::string description = conf->getParamDescr(param);
        if (!description.empty()) {
            std::cout << "  Description: " << description << "\n";
        }
        std::cout << "\n";
    }
}

void SimpleConfigurator::show() {
    running = true;
    while (running) {
        displayMenu();
        processUserInput();
    }
}

void SimpleConfigurator::close() {
    running = false;
}

void SimpleConfigurator::refresh() {
    // Clear screen in a platform-independent way
    std::cout << std::string(50, '\n');
    displayMenu();
}

void SimpleConfigurator::displayMenu() {
    std::cout << "\n=== Configurator Menu ===\n"
              << "1. List Configurables\n"
              << "2. Modify Parameter\n"
              << "3. Save to File\n"
              << "4. Load from File\n"
              << "5. Help\n"
              << "0. Exit\n"
              << "Choice: ";
}


// Improved implementation of existing methods
void SimpleConfigurator::processUserInput() {
    std::string input;
    std::getline(std::cin, input);
    
    if (input.empty()) return;
    
    switch (input[0]) {
        case '1': listConfigurables(); break;
        case '2': modifyParameter(); break;
        case '3': saveToFile(); break;
        case '4': loadFromFile(); break;
        case '5': displayHelp(); break;
        case '0': close(); break;
        default: 
            std::cout << "Invalid choice. Press '5' for help.\n";
    }
}


void SimpleConfigurator::listConfigurables() {
    refresh();
    std::cout << "=== Configurables List ===\n\n";
    
    for(ConfigurableList::iterator it = configList.begin(); it != configList.end(); ++it) {
        displayConfigurable(*it);
    }
    
    std::cout << "\nPress Enter to continue...";
    std::cin.get();
}

void SimpleConfigurator::displayConfigurable(Configurable* conf, int indent) {
    if (!conf) return;
    
    std::string indentation = getIndentation(indent);
    std::cout << indentation << "- " << conf->getName() << "\n";
    
    // Display parameters
    std::list<Configurable::paramkey> params = conf->getAllParamNames();
    for (const auto& param : params) {
        double value = conf->getParam(param);
        std::cout << indentation << "  * " << param << " = " << value;
        
        // Show bounds if they exist
        if (conf->hasParamvalBounds(param)) {
            Configurable::paramvalBounds bounds = conf->getParamvalBounds(param);
            if (bounds.first != bounds.second) {
                std::cout << " (range: " << bounds.first << " to " << bounds.second << ")";
            }
        } else if (conf->hasParamintBounds(param)) {
            Configurable::paramintBounds bounds = conf->getParamintBounds(param);
            if (bounds.first != bounds.second) {
                std::cout << " (range: " << bounds.first << " to " << bounds.second << ")";
            }
        }
        std::cout << "\n";
    }
    
    // Display child configurables
    const Configurable::configurableList& children = conf->getConfigurables();
    for (const auto& child : children) {
        displayConfigurable(child, indent + 2);
    }
}

void SimpleConfigurator::modifyParameter() {
    refresh();
    std::cout << "Enter Configurable ID: ";
    std::string idStr;
    std::getline(std::cin, idStr);
    int id = std::atoi(idStr.c_str());
    
    // Find configurable
    Configurable* conf = nullptr;
    for(ConfigurableList::iterator it = configList.begin(); 
        it != configList.end(); ++it) {
        if((*it)->getId() == id) {
            conf = *it;
            break;
        }
    }
    
    if(!conf) {
        std::cout << "Configurable not found!\n";
        return;
    }
    
    displayParameterDetails(conf);
    
    std::cout << "Enter parameter name: ";
    std::string paramName;
    std::getline(std::cin, paramName);
    
    std::cout << "Enter new value: ";
    std::string value;
    std::getline(std::cin, value);
    
    if(setConfigurableParameter(conf, paramName, value)) {
        std::cout << "Parameter updated successfully!\n";
    } else {
        std::cout << "Failed to update parameter!\n";
    }
}

bool SimpleConfigurator::setConfigurableParameter(Configurable* conf, 
    const std::string& paramName, const std::string& value) {
    if (!conf) return false;
    
    try {
        double dVal = std::stod(value);
        
        // Check both types of bounds
        if (conf->hasParamvalBounds(paramName)) {
            Configurable::paramvalBounds bounds = conf->getParamvalBounds(paramName);
            if (bounds.first != bounds.second && (dVal < bounds.first || dVal > bounds.second)) {
                std::cout << "Value out of bounds. Valid range (double): [" 
                         << bounds.first << " to " << bounds.second << "]\n";
                return false;
            }
        } else if (conf->hasParamintBounds(paramName)) {
            Configurable::paramintBounds bounds = conf->getParamintBounds(paramName);
            if (bounds.first != bounds.second && (dVal < bounds.first || dVal > bounds.second)) {
                std::cout << "Value out of bounds. Valid range (int): [" 
                         << bounds.first << " to " << bounds.second << "]\n";
                return false;
            }
        }
        
        return conf->setParam(paramName, dVal);
    } catch (const std::exception& e) {
        std::cout << "Error setting parameter: " << e.what() << "\n";
        return false;
    }
}

void SimpleConfigurator::saveToFile() {
    std::cout << "Enter filename: ";
    std::string filename;
    std::getline(std::cin, filename);
    
    for(ConfigurableList::iterator it = configList.begin(); 
        it != configList.end(); ++it) {
        (*it)->storeCfg(filename.c_str());
    }
    std::cout << "Configuration saved!\n";
}

void SimpleConfigurator::loadFromFile() {
    std::cout << "Enter filename: ";
    std::string filename;
    std::getline(std::cin, filename);
    
    for(ConfigurableList::iterator it = configList.begin(); 
        it != configList.end(); ++it) {
        (*it)->restoreCfg(filename.c_str());
    }
    std::cout << "Configuration loaded!\n";
}

void SimpleConfigurator::displayHelp() {
    refresh();
    std::cout << "=== Help ===\n"
              << "This configurator allows you to:\n"
              << "- View all configurables and their parameters\n"
              << "- Modify parameter values\n"
              << "- Save/load configurations to/from files\n"
              << "\nPress Enter to continue...";
    std::cin.get();
}

std::string SimpleConfigurator::getIndentation(int level) const {
    return std::string(level * 2, ' ');
}

} // namespace lpzrobots