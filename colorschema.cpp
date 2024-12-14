// colorschema.cpp - VSG Version

#include "colorschema.h"
#include <cassert>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <algorithm>

namespace lpzrobots {

    const Color Color::undef(0.0f, 0.0f, 0.0f, 0.0f);

    ColorSchema::ColorSchema()
        : dummy(1.0f, 1.0f, 1.0f, 1.0f) {
    }

    Color ColorSchema::color(const std::string& name_or_id_or_alias) const {
        return color(name_or_id_or_alias, 0);
    }

    Color ColorSchema::color(const std::string& name_or_id_or_alias, int alias_set) const {
        Color c;
        if (color(c, name_or_id_or_alias, alias_set)) {
            return c;
        } else {
            return dummy;
        }
    }

    bool ColorSchema::color(Color& color, const std::string& name_or_id_or_alias,
                            int alias_set) const {
        if (getColor(color, name_or_id_or_alias)) {
            return true;
        } else {
            AliasMap::const_iterator i = aliases.find(name_or_id_or_alias);
            if (i == aliases.end()) {
                return false;
            } else { // We know this alias
                const AliasVector& v = i->second;
                if (static_cast<int>(v.size()) > alias_set && !v[alias_set].empty()) {
                    return getColor(color, v[alias_set]);
                } else { // Don't have the alias in the set
                    if (!v.empty())
                        return getColor(color, v[0]); // Try alias-set 0
                    else
                        return false;
                }
            }
        }
    }

    std::string ColorSchema::getLoadErrorString(int value) const {
        switch (value) {
        case 0: return "No colors/aliases found";
        case -1:
            return "Could not find file";
        case -2:
            return "Could not open file for reading";
        case -3:
            return "Parse error";
        case -4:
            return "Columns line not found or unsupported number (0,1)";
        default: return "No error";
        }
    }

    int ColorSchema::loadPalette(const std::string& gplfilename) {
        std::ifstream file(gplfilename);
        if (!file.is_open()) {
            return -2; // Could not open file
        }

        std::string line;
        int columns = 0;
        // Skip lines until we find "Columns" or "#"
        while (std::getline(file, line)) {
            if (line.find("Columns") != std::string::npos) {
                if (sscanf(line.c_str(), "Columns: %i", &columns) != 1)
                    return -4; // Columns line not found or unsupported
            }
            if (line[0] == '#') {
                break;
            }
        }

        int r, g, b;
        int count = 0;
        if (columns == 0) {
            while (std::getline(file, line)) {
                if (line.empty() || line[0] == '#')
                    continue;
                char name[1024];
                if (sscanf(line.c_str(), "%i %i %i %s", &r, &g, &b, name) == 4) {
                    addColor(Color::rgb255(r, g, b), std::string(name));
                    count++;
                }
            }
        } else if (columns == 1) {
            while (std::getline(file, line)) {
                if (line.empty() || line[0] == '#')
                    continue;
                char name1[1024], name2[1024];
                if (sscanf(line.c_str(), "%i %i %i %s %s", &r, &g, &b, name1, name2) == 5) {
                    addColor(Color::rgb255(r, g, b), std::string(name1));
                    addColor(Color::rgb255(r, g, b), std::string(name2));
                    count++;
                }
            }
        } else {
            std::cerr << "Cannot read GPL file with " << columns
                      << " name columns, support 0 or 1" << std::endl;
            return -4;
        }
        file.close();
        return count > 0 ? count : 0;
    }

    int ColorSchema::loadAliases(const std::string& filename, int alias_set_offset) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            return -2; // Could not open file
        }

        std::string line;
        int count = 0;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#')
                continue;
            std::istringstream iss(line);
            std::string alias, name;
            int alias_set = 0;
            if (iss >> alias >> name >> alias_set) {
                if (addAlias(alias, name, alias_set + alias_set_offset)) {
                    count++;
                }
            } else if (iss >> alias >> name) {
                if (addAlias(alias, name, alias_set_offset)) {
                    count++;
                }
            }
        }
        file.close();
        return count > 0 ? count : 0;
    }

    void ColorSchema::addColor(const Color& color, const std::string& name) {
        colors[name] = color;
    }

    bool ColorSchema::addAlias(const std::string& alias, const std::string& name,
                               int alias_set) {
        assert(alias_set >= 0);
        if (existsColor(alias)) {
            std::cerr << "Cannot add alias '" << alias << "' because a color with that name exists" << std::endl;
            return false;
        }
        if (!existsColor(name)) {
            std::cerr << "Cannot add alias '" << alias << "' to '" << name
                      << "' because no color with that name exists" << std::endl;
            return false;
        }
        AliasVector& v = aliases[alias];
        if (static_cast<int>(v.size()) <= alias_set) {
            v.resize(alias_set + 1);
        }
        v[alias_set] = name;
        return true;
    }

    void ColorSchema::setDefaultColor(const Color& c) {
        dummy = c;
    }

    const Color& ColorSchema::getDefaultColor() const {
        return dummy;
    }

    bool ColorSchema::existsColor(const std::string& name) const {
        return colors.find(name) != colors.end();
    }

    void ColorSchema::print(std::ostream& out) const {
        out << "Colors:\n";
        for (const auto& c : colors) {
            out << std::setw(20) << c.first << ": " << c.second << std::endl;
        }
        out << "Aliases:\n";
        for (const auto& a : aliases) {
            const AliasVector& v = a.second;
            out << std::setw(20) << a.first << ": ";
            for (const auto& alias_name : v) {
                out << alias_name << ",\t";
            }
            out << std::endl;
        }
    }

    bool ColorSchema::getColor(Color& c, const std::string& name) const {
        auto it = colors.find(name);
        if (it != colors.end()) {
            c = it->second;
            return true;
        } else {
            return false;
        }
    }

}
