#pragma once

#include <atomic>
#include <cstdint>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>

#include <glm/glm.hpp>

class IniFileParser
{
  public:
    // Construct IniFileParser and parse given filename. See ini.h for more info about the parsing.
    IniFileParser();
    IniFileParser(const std::string& filename);
    bool parseFile(const std::string& filename);

    // Return the result of ini_parse(), i.e., 0 on success, line number of first error on parse error, or -1 on file
    // open error.
    int parseError() const;

    // Get a string value from INI file, returning default_value if not found.
    std::string getString(const std::string& section, const std::string& name, const std::string& default_value) const;
    void        readString(const std::string& section, const std::string& name, std::string& value) const;

    // Get an integer (long) value from INI file, returning default_value if not found or not a valid integer (decimal
    // "1234", "-1234", or hex "0x4d2").
    long getInteger(const std::string& section, const std::string& name, long default_value) const;
    void readSize(const std::string& sectionName, const std::string& variableName, size_t& value) const;

    // Get an integter value
    template <typename T>
    void readInteger(const std::string& sectionName, const std::string& variableName, T& value) const
    {
        std::istringstream stream(getString(sectionName, variableName, ""));
        stream >> std::setbase(0) >> value;
    }

    template <>
    void readInteger<uint8_t>(const std::string& sectionName, const std::string& variableName, uint8_t& value) const
    {
        // Use a temporary value of uint16_t, since uint8_t would be treated as a character type by istringstream.
        uint16_t tempValue = value;
        readInteger(sectionName, variableName, tempValue);
        value = static_cast<uint8_t>(tempValue);
    }

    template <>
    void readInteger<int8_t>(const std::string& sectionName, const std::string& variableName, int8_t& value) const
    {
        // Use a temporary value of int16_t, since int8_t would be treated as a character type by istringstream.
        int16_t tempValue = value;
        readInteger(sectionName, variableName, tempValue);
        value = static_cast<int8_t>(tempValue);
    }

    // Get an enumeration value
    template <typename T>
    T getEnum(const std::string& sectionName, const std::string& variableName, const T& default_value) const
    {
        return T(getInteger(sectionName.c_str(), variableName, static_cast<int>(default_value)));
    }

    template <typename T>
    void readEnum(const std::string& sectionName, const std::string& variableName, T& value) const
    {
        value = T(getInteger(sectionName.c_str(), variableName, static_cast<int>(value)));
    }

    // Get a real (floating point double) value from INI file, returning default_value if not found or not a valid
    // floating point value according to strtod().
    double getReal(const std::string& section, const std::string& name, double default_value) const;
    void   readScalar(const std::string& sectionName, const std::string& variableName, float& value) const;
    void   readScalar(const std::string& sectionName, const std::string& variableName, double& value) const;

    // Get a boolean value from INI file, returning default_value if not found or if not a valid true/false value. Valid
    // true values are "true", "yes", "on", "1", and valid false values are "false", "no", "off", "0" (not case
    // sensitive).
    bool getBoolean(const std::string& section, const std::string& name, bool default_value) const;
    void readBoolean(const std::string& sectionName, const std::string& variableName, bool& value) const;
    void readAtomicBoolean(const std::string& sectionName,
                           const std::string& variableName,
                           std::atomic<bool>& value) const;

    // Get a single precision vector from INI file using strtod().
    bool getVector(const std::string& section, const std::string& name, glm::vec2& outputVec) const;

    // This method get the full filename of the INI file being parsed
    std::string getFullFilename() const;

  private:
    std::string                        mFilename;
    int                                mError;
    std::map<std::string, std::string> mValues;
    void                               setFilename(const std::string& filename);
    static std::string                 makeKey(const std::string& section, const std::string& name);
    static int valueHandler(void* user, const char* section, const char* name, const char* value);
};
