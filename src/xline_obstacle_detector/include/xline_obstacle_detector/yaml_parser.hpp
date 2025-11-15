#pragma once
#include <string>
#include <fstream>
#include <unordered_map>
#include <sstream>
#include <vector>
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <type_traits>

namespace xline
{
namespace YamlParser
{

class YamlParser
{
public:
  explicit YamlParser(const std::string& file_path)
  {
    std::ifstream file(file_path);
    if (!file.is_open())
    {
      throw std::runtime_error("Failed to open YAML file: " + file_path);
    }

    std::string line;
    std::string current_prefix;  // 用于嵌套键的前缀
    while (std::getline(file, line))
    {
      line = trim(line);

      // 忽略空行或注释行
      if (line.empty() || line[0] == '#')
      {
        continue;
      }

      if (isNestedKey(line))
      {
        // 处理嵌套键
        current_prefix = parseKey(line);
      }
      else
      {
        // 普通键值对
        auto [key, value] = parseKeyValue(line);
        if (!current_prefix.empty())
        {
          key = current_prefix + "." + key;  // 添加前缀
        }
        yaml_data_[key] = value;
      }
    }
    file.close();
  }

  // 获取指定参数值 - 通用模板
  template <typename T>
  T getParameter(const std::string& key) const
  {
    auto it = yaml_data_.find(key);
    if (it == yaml_data_.end())
    {
      throw std::runtime_error("Parameter " + key + " not found in the YAML file.");
    }

    std::istringstream iss(it->second);
    T value;
    iss >> value;

    if (iss.fail())
    {
      throw std::runtime_error("Failed to convert parameter " + key + " to the requested type.");
    }

    return value;
  }

  // 检查参数是否存在
  bool hasParameter(const std::string& key) const
  {
    return yaml_data_.find(key) != yaml_data_.end();
  }

  // 获取所有键
  std::vector<std::string> getAllKeys() const
  {
    std::vector<std::string> keys;
    keys.reserve(yaml_data_.size());
    for (const auto& [key, _] : yaml_data_)
    {
      keys.push_back(key);
    }
    return keys;
  }

  // 打印所有键值对
  void printAll() const
  {
    for (const auto& [key, value] : yaml_data_)
    {
      std::cout << key << ": " << value << std::endl;
    }
  }

private:
  std::unordered_map<std::string, std::string> yaml_data_;

  // 检查是否是嵌套键
  bool isNestedKey(const std::string& line) const
  {
    return line.back() == ':' && line.find(':') == line.size() - 1;
  }

  // 解析嵌套键
  std::string parseKey(const std::string& line) const
  {
    return trim(line.substr(0, line.size() - 1));
  }

  // 解析键值对
  std::pair<std::string, std::string> parseKeyValue(const std::string& line) const
  {
    auto delimiter_pos = line.find(':');
    if (delimiter_pos == std::string::npos)
    {
      throw std::runtime_error("Invalid YAML format: " + line);
    }
    std::string key = trim(line.substr(0, delimiter_pos));
    std::string value = trim(line.substr(delimiter_pos + 1));
    return { key, value };
  }

  // 去除字符串首尾空格
  std::string trim(const std::string& str) const
  {
    const auto start = str.find_first_not_of(" \t");
    const auto end = str.find_last_not_of(" \t");
    return (start == std::string::npos) ? "" : str.substr(start, end - start + 1);
  }
};

// 为bool类型特化模板
template <>
inline bool YamlParser::getParameter<bool>(const std::string& key) const
{
  auto it = yaml_data_.find(key);
  if (it == yaml_data_.end())
  {
    throw std::runtime_error("Parameter " + key + " not found in the YAML file.");
  }

  std::string value = it->second;
  // 转换为小写
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) { return std::tolower(c); });

  if (value == "true" || value == "yes" || value == "1" || value == "on")
  {
    return true;
  }
  else if (value == "false" || value == "no" || value == "0" || value == "off")
  {
    return false;
  }
  else
  {
    throw std::runtime_error("Invalid boolean value for parameter " + key + ": " + it->second);
  }
}

// 为string类型特化模板
template <>
inline std::string YamlParser::getParameter<std::string>(const std::string& key) const
{
  auto it = yaml_data_.find(key);
  if (it == yaml_data_.end())
  {
    throw std::runtime_error("Parameter " + key + " not found in the YAML file.");
  }

  std::string value = it->second;
  // 移除可能的引号
  if ((value.front() == '"' && value.back() == '"') || (value.front() == '\'' && value.back() == '\''))
  {
    return value.substr(1, value.length() - 2);
  }
  return value;
}

// 为vector<double>类型特化模板
template <>
inline std::vector<double> YamlParser::getParameter<std::vector<double>>(const std::string& key) const
{
  auto it = yaml_data_.find(key);
  if (it == yaml_data_.end())
  {
    throw std::runtime_error("Parameter " + key + " not found in the YAML file.");
  }

  std::vector<double> result;
  std::string value = it->second;

  // 移除可能的方括号
  if (value.front() == '[' && value.back() == ']')
  {
    value = value.substr(1, value.length() - 2);
  }

  std::istringstream iss(value);
  std::string token;
  while (std::getline(iss, token, ','))
  {
    token = trim(token);
    if (!token.empty())
    {
      try
      {
        result.push_back(std::stod(token));
      }
      catch (const std::exception& e)
      {
        throw std::runtime_error("Failed to convert array element to double in parameter " + key);
      }
    }
  }

  return result;
}

// 为vector<int>类型特化模板
template <>
inline std::vector<int> YamlParser::getParameter<std::vector<int>>(const std::string& key) const
{
  auto it = yaml_data_.find(key);
  if (it == yaml_data_.end())
  {
    throw std::runtime_error("Parameter " + key + " not found in the YAML file.");
  }

  std::vector<int> result;
  std::string value = it->second;

  // 移除可能的方括号
  if (value.front() == '[' && value.back() == ']')
  {
    value = value.substr(1, value.length() - 2);
  }

  std::istringstream iss(value);
  std::string token;
  while (std::getline(iss, token, ','))
  {
    token = trim(token);
    if (!token.empty())
    {
      try
      {
        result.push_back(std::stoi(token));
      }
      catch (const std::exception& e)
      {
        throw std::runtime_error("Failed to convert array element to int in parameter " + key);
      }
    }
  }

  return result;
}

// 为vector<string>类型特化模板
template <>
inline std::vector<std::string> YamlParser::getParameter<std::vector<std::string>>(const std::string& key) const
{
  auto it = yaml_data_.find(key);
  if (it == yaml_data_.end())
  {
    throw std::runtime_error("Parameter " + key + " not found in the YAML file.");
  }

  std::vector<std::string> result;
  std::string value = it->second;

  // 移除可能的方括号
  if (value.front() == '[' && value.back() == ']')
  {
    value = value.substr(1, value.length() - 2);
  }

  std::istringstream iss(value);
  std::string token;
  while (std::getline(iss, token, ','))
  {
    token = trim(token);
    // 移除可能的引号
    if ((token.front() == '"' && token.back() == '"') || (token.front() == '\'' && token.back() == '\''))
    {
      token = token.substr(1, token.length() - 2);
    }
    if (!token.empty())
    {
      result.push_back(token);
    }
  }

  return result;
}

}  // namespace YamlParser
}  // namespace xline