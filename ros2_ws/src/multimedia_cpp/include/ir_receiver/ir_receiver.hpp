#pragma once

#include <string>
#include <unordered_map>

class IRMap
{
public:
    static std::string translate(const std::string &key)
    {
        // Map LIRC key names to your command strings
        static const std::unordered_map<std::string, std::string> table = {
            {"KEY_VOLUMEUP",   "volume_up"},
            {"KEY_VOLUMEDOWN", "volume_down"},
            {"KEY_PLAYPAUSE",  "play_pause"},
            {"KEY_NEXT",       "next"},
            {"KEY_PREVIOUS",   "previous"},
            {"KEY_POWER",      "power"},
        };

        if (auto it = table.find(key); it != table.end())
            return it->second;

        // default: pass through key name
        return key;
    }
};