// colorschema.h - VSG Version

#ifndef __COLORSCHEMA_H
#define __COLORSCHEMA_H

#include "color.h"
#include <string>
#include <vector>
#include <unordered_map>
#include <iostream>

namespace lpzrobots {
    /**
       A store for colors with a set of aliases.
       The alias-sets are numbered where the 0'th plays the role of a default set.
     */
    class ColorSchema
    {
    public:
        typedef std::unordered_map<std::string, Color> ColorMap;
        typedef std::vector<std::string> AliasVector;
        typedef std::unordered_map<std::string, AliasVector> AliasMap;

        ColorSchema();

        /** Retrieves a color with the given name/id/alias
            If no color is found that matches the id/alias then
            the default color is returned.
            Always the alias-set 0 is checked first
        */
        Color color(const std::string& name_or_id_or_alias) const;

        /** Retrieves a color with the given name/id/alias from given alias_set
            If not found then the default alias_set (0) is checked
         */
        Color color(const std::string& name_or_id_or_alias, int alias_set) const;

        /** Call by reference version
            Returns false if color not found
        */
        bool color(Color& color, const std::string& name_or_id_or_alias,
                   int alias_set = 0) const;

        /// Checks whether color with the name exists (no aliases are checked)
        bool existsColor(const std::string& name) const;

        /** Loads a GPL (GIMP palette file) and returns the number of loaded colors
            The names of the colors should not contain white spaces!
         */
        int loadPalette(const std::string& gplfilename);

        /** Loads aliases from text file with lines containing:
            aliasname colorname/id [alias-set]
            @param alias_set_offset Number that is added to the alias_set number in the file
         */
        int loadAliases(const std::string& filename, int alias_set_offset = 0);

        /** Adds a color to the color store
            (to add the id call the function twice with id as name)
        */
        void addColor(const Color& color, const std::string& name);

        /** Adds a color alias (into the given alias-set)
            @param name Name/id of existing color
            @param alias New name
            @return true if alias was stored or
             false if color name does not exist or
                   alias names a color and is therefore rejected
        */
        bool addAlias(const std::string& alias, const std::string& name, int alias_set = 0);

        void setDefaultColor(const Color& c);
        const Color& getDefaultColor() const;

        /// Returns error string for value returned by loadPalette and loadAliases
        std::string getLoadErrorString(int value) const;

        /// Prints all colors and aliases
        void print(std::ostream& out) const;

    protected:
        // Only name/id, no alias checking
        bool getColor(Color& color, const std::string& name) const;

    private:
        Color dummy;       // Default color
        ColorMap colors;   // Map of color names to Color objects
        AliasMap aliases;  // Map of aliases to color names
    };
}

#endif // __COLORSCHEMA_H
