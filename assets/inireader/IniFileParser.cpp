
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <stdexcept>

#include "Shlwapi.h"
#include "libloaderapi.h"

#include "ini.h"
#include "IniFileParser.h"

IniFileParser::IniFileParser()
{
}

IniFileParser::IniFileParser(const std::string& filename)
{
    bool parseSuccessful = parseFile(filename);
    if (!parseSuccessful)
    {
        printf("Error parsing %s\n", mFilename.c_str());
    }
}

bool IniFileParser::parseFile(const std::string& filename)
{
    setFilename(filename);
    mError = ini_parse(mFilename.c_str(), valueHandler, this);
    return mError == 0;
}

int IniFileParser::parseError() const
{
    return mError;
}

std::string IniFileParser::getString(const std::string& section,
                                     const std::string& name,
                                     const std::string& default_value) const
{
    std::string key = makeKey(section, name);
    return mValues.count(key) ? mValues.at(key) : default_value;
}
void IniFileParser::readString(const std::string& section, const std::string& name, std::string& value) const
{
    value = getString(section, name, value);
}

long IniFileParser::getInteger(const std::string& section, const std::string& name, long default_value) const
{
    std::string valstr = getString(section, name, "");
    const char* value  = valstr.c_str();
    char*       end;
    // This parses "1234" (decimal) and also "0x4D2" (hex)
    long n = strtol(value, &end, 0);
    return end > value ? n : default_value;
}

void IniFileParser::readSize(const std::string& sectionName, const std::string& variableName, size_t& value) const
{
    readInteger(sectionName, variableName, value);
}

double IniFileParser::getReal(const std::string& section, const std::string& name, double default_value) const
{
    std::string valstr = getString(section, name, "");
    const char* value  = valstr.c_str();
    char*       end;
    double      n = strtod(value, &end);
    return end > value ? n : default_value;
}

void IniFileParser::readScalar(const std::string& sectionName, const std::string& variableName, float& value) const
{
    value = static_cast<float>(getReal(sectionName.c_str(), variableName, value));
}

void IniFileParser::readScalar(const std::string& sectionName, const std::string& variableName, double& value) const
{
    value = getReal(sectionName.c_str(), variableName, value);
}

bool IniFileParser::getBoolean(const std::string& section, const std::string& name, bool default_value) const
{
    std::string valstr = getString(section, name, "");
    // Convert to lower case to make string comparisons case-insensitive
    std::transform(valstr.begin(), valstr.end(), valstr.begin(), ::tolower);
    if (valstr == "true" || valstr == "yes" || valstr == "on" || valstr == "1")
        return true;
    else if (valstr == "false" || valstr == "no" || valstr == "off" || valstr == "0")
        return false;
    else
        return default_value;
}

void IniFileParser::readBoolean(const std::string& sectionName, const std::string& variableName, bool& value) const
{
    value = getBoolean(sectionName, variableName, value);
}

void IniFileParser::readAtomicBoolean(const std::string& sectionName,
                                      const std::string& variableName,
                                      std::atomic<bool>& value) const
{
    value = getBoolean(sectionName, variableName, value);
}

bool IniFileParser::getVector(const std::string& section, const std::string& name, glm::vec2& outputVec) const
{
    std::string valstr = getString(section, name, "");
    if (valstr.length() > 0)
    {
        size_t idx   = valstr.find(",");
        outputVec.x = static_cast<float>(atof(valstr.substr(0, idx).c_str()));
        outputVec.y = static_cast<float>(atof(valstr.substr(idx + 1, valstr.length() - idx).c_str()));
        return true;
    }
    else
    {
        return false;
    }
}

void IniFileParser::setFilename(const std::string& filename)
{
    // Get the path of the current module (i.e. who's calling this static library)
    char configurationFilePath[MAX_PATH];
    GetModuleFileNameA(NULL, configurationFilePath, sizeof(configurationFilePath));

    mFilename = filename;
}

std::string IniFileParser::makeKey(const std::string& section, const std::string& name)
{
    std::string key = section + "=" + name;
    // Convert to lower case to make section/name lookups case-insensitive
    std::transform(key.begin(), key.end(), key.begin(), ::tolower);
    return key;
}

int IniFileParser::valueHandler(void* user, const char* section, const char* name, const char* value)
{
    IniFileParser* reader = (IniFileParser*) user;
    std::string    key    = makeKey(section, name);
    if (reader->mValues[key].size() > 0)
    {
        printf("[IniFileParser] Found multiple definitions of parameter \"%s\" within section \"%s\" of %s; using only "
               "the first value.\n",
               name,
               section,
               reader->mFilename.c_str());
    }
    else
    {
        reader->mValues[key] += value;
    }
    return 1;
}

std::string IniFileParser::getFullFilename() const
{
    return mFilename;
}
