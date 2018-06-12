/*
  This file is part of bitplanes.

  bitplanes is free software: you can redistribute it and/or modify
  it under the terms of the Lesser GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  bitplanes is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  Lesser GNU General Public License for more details.

  You should have received a copy of the Lesser GNU General Public License
  along with bitplanes.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef BP_UTIL_CONFIG_FILE_H
#define BP_UTIL_CONFIG_FILE_H

#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <map>

#include "bitplanes/utils/error.h"
#include "bitplanes/utils/memory.h"
#include "bitplanes/utils/str2num.h"
#include "bitplanes/utils/icompare.h"

namespace bp {

/**
 * simple config file with data of the form
 *   VarName = Value
 *
 * Lines that begin with a '#' or '%' are treated as comments
 *
 * Example usage
 *
 *    ConfigFile cf("myfile.cfg");
 *
 *    // get a variable with a default value if it does not exist in myfile.cfg
 *    auto v = cf.get<std::string>("MyVariable", "default");
 *
 *    // this will throw an error if the variable does not exist
 *    try {
 *      auto required_var = cf.get<int>("VariableName");
 *    } catch(const std::exception& ex) {
 *      std::cerr << "value 'VariableName' is required\n";
 *    }
 *
 *    // We can also print the contents of the ConfigFile
 *    std::cout << cf << std::endl;
 *
 *    // or add new values
 *    cf.set<double>("MyNewVariable", 1.618);
 *
 *    // and write it to disk
 *    cf.save("newfile.cfg");
 *
 */
class ConfigFile
{
 public:
  typedef SharedPointer<ConfigFile> Pointer;

 public:
  /**
   * default constructor, does not do anything
   */
  ConfigFile();

  /**
   * Loads a config file from 'filename'.
   *
   * \throw Error if filename does not exist
   */
  ConfigFile(std::string filename);

  /**
   * Loads the config from an opened ifstream
   * \throw Error if 'ifs' is not open
   */
  ConfigFile(std::ifstream& ifs);

  /**
   * Writes the contents of the file to disk
   *
   * \param filename output filename
   * \return true if successfull
   */
  bool save(std::string filename) const;

  /**
   * Get the value named 'var_name'
   *
   * \throw Error if 'var_name' does not exist, or conversion to the required
   * type 'T' fails
   */
  template <typename T> inline
  T get(std::string var_name) const;

  /**
   * Get the value name 'var_name'
   *
   * If any error occurs (e.g. var_name does not exist) the function will
   * silently return the supplied 'default_val'
   */
  template <typename T> inline
  T get(std::string var_name, const T& default_val) const;

  /**
   * Sets 'var_name' to the specified value
   */
  template <typename T> inline
  ConfigFile& set(std::string var_name, const T& value);

  /**
   * sets values with method chaining. For example,
   *
   * ConfigFile cf;
   * cf("SpeedOfLight", "299792458.0")
   *   ("PI",           "3.14159265359")
   *   ("PHI",          "1.618033988749895").save("my_awesome_config.cfg");
   */
  ConfigFile& operator()(const std::string&, const std::string&);

  friend std::ostream& operator<<(std::ostream&, const ConfigFile&);

 protected:
  void parse(std::ifstream&);

  std::map<std::string, std::string, CaseInsenstiveComparator> _data;
}; // ConfigFile


template <typename T>
T ConfigFile::get(std::string name) const
{
  const auto& value_it = _data.find(name);
  if(value_it == _data.end())
    throw Error("no key " + name);

  T ret;
  if(!str2num(value_it->second, ret))
    throw Error("failed to convert '" + value_it->second +
                "' to type " + typeid(T).name());

  return ret;
}

template <typename T>
T ConfigFile::get(std::string name, const T& default_val) const
{
  try {
    return get<T>(name);
  } catch(const std::exception& ex) {
    if(0) std::cerr<<"config_file: get("<<name<<"): error: "<<ex.what()<<std::endl;
    return default_val;
  }
}

template <typename T> inline
ConfigFile& ConfigFile::set(std::string name, const T& value)
{
  _data[name] = std::to_string(value);
  return *this;
}

template<> inline
ConfigFile& ConfigFile::set(std::string name, const bool& value)
{
  _data[name] = std::to_string(static_cast<int>(value));
  return *this;
}


}; // bp

#endif // TT_UTIL_CONFIG_FILE_H
