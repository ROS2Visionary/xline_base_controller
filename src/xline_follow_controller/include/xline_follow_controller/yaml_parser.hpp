#pragma once
#include <string>
#include <fstream>
#include <unordered_map>
#include <sstream>
#include <vector>
#include <stdexcept>
#include <algorithm>
#include <type_traits>
#include <rclcpp/rclcpp.hpp>

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
    std::vector<std::string> prefix_stack;  // 用于多级嵌套的前缀栈
    std::vector<int> indent_stack;          // 对应的缩进级别栈
    
    while (std::getline(file, line))
    {
      // 跳过空行和纯注释行
      std::string trimmed_line = trim(line);
      if (trimmed_line.empty() || trimmed_line[0] == '#')
      {
        continue;
      }

      int current_indent = getIndentLevel(line);

      // 根据缩进调整前缀栈
      adjustPrefixStack(prefix_stack, indent_stack, current_indent);

      if (isNestedKey(trimmed_line))
      {
        // 处理嵌套键：添加到前缀栈
        std::string key = parseKey(trimmed_line);
        prefix_stack.push_back(key);
        indent_stack.push_back(current_indent);
      }
      else if (hasKeyValuePair(trimmed_line))
      {
        // 普通键值对
        auto [key, value] = parseKeyValue(trimmed_line);
        
        // 构建完整的键路径
        std::string full_key = buildFullKey(prefix_stack, key);
        yaml_data_[full_key] = value;
      }
      // 其他情况（如纯值行、数组等）暂时忽略
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

  // 打印所有键值对 (使用ROS2日志)
  void printAll(const rclcpp::Logger& logger) const
  {
    for (const auto& [key, value] : yaml_data_)
    {
      RCLCPP_INFO(logger, "%s: %s", key.c_str(), value.c_str());
    }
  }

  // 调试：打印特定前缀的所有键 (使用ROS2日志)
  void printKeysWithPrefix(const std::string& prefix, const rclcpp::Logger& logger) const
  {
    RCLCPP_INFO(logger, "Keys with prefix '%s':", prefix.c_str());
    for (const auto& [key, value] : yaml_data_)
    {
      if (key.find(prefix) == 0)
      {
        RCLCPP_INFO(logger, "  %s: %s", key.c_str(), value.c_str());
      }
    }
  }

private:
  std::unordered_map<std::string, std::string> yaml_data_;

  // 检查是否包含键值对
  bool hasKeyValuePair(const std::string& line) const
  {
    auto colon_pos = line.find(':');
    if (colon_pos == std::string::npos)
    {
      return false;
    }
    
    // 获取冒号后的内容（移除注释）
    std::string value_part = line.substr(colon_pos + 1);
    auto comment_pos = value_part.find('#');
    if (comment_pos != std::string::npos)
    {
      value_part = value_part.substr(0, comment_pos);
    }
    value_part = trim(value_part);
    
    // 如果冒号后有非空内容，则为键值对
    return !value_part.empty();
  }

  // 检查是否是嵌套键
  bool isNestedKey(const std::string& line) const
  {
    auto colon_pos = line.find(':');
    if (colon_pos == std::string::npos)
    {
      return false;
    }
    
    // 获取冒号后的内容（移除注释）
    std::string value_part = line.substr(colon_pos + 1);
    auto comment_pos = value_part.find('#');
    if (comment_pos != std::string::npos)
    {
      value_part = value_part.substr(0, comment_pos);
    }
    value_part = trim(value_part);
    
    // 如果冒号后没有内容，则为嵌套键
    return value_part.empty();
  }

  // 解析嵌套键
  std::string parseKey(const std::string& line) const
  {
    auto colon_pos = line.find(':');
    if (colon_pos == std::string::npos)
    {
      return "";
    }
    return trim(line.substr(0, colon_pos));
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
    
    // 移除行内注释
    auto comment_pos = value.find('#');
    if (comment_pos != std::string::npos)
    {
      value = trim(value.substr(0, comment_pos));
    }
    
    return { key, value };
  }

  // 去除字符串首尾空格
  std::string trim(const std::string& str) const
  {
    const auto start = str.find_first_not_of(" \t");
    const auto end = str.find_last_not_of(" \t");
    return (start == std::string::npos) ? "" : str.substr(start, end - start + 1);
  }

  // 获取行的缩进级别
  int getIndentLevel(const std::string& line) const
  {
    int indent = 0;
    for (char c : line)
    {
      if (c == ' ')
      {
        indent++;
      }
      else if (c == '\t')
      {
        indent += 2;  // 制表符计为2个空格
      }
      else
      {
        break;
      }
    }
    return indent;
  }

  // 根据当前缩进调整前缀栈
  void adjustPrefixStack(std::vector<std::string>& prefix_stack, 
                        std::vector<int>& indent_stack, 
                        int current_indent) const
  {
    // 移除缩进级别大于等于当前级别的前缀
    // 注意：只有当前缩进小于等于栈顶缩进时才移除，允许多级嵌套
    while (!indent_stack.empty() && indent_stack.back() >= current_indent)
    {
      prefix_stack.pop_back();
      indent_stack.pop_back();
    }
  }

  // 构建完整的键路径
  std::string buildFullKey(const std::vector<std::string>& prefix_stack, 
                          const std::string& key) const
  {
    if (prefix_stack.empty())
    {
      return key;
    }
    
    std::string full_key;
    for (size_t i = 0; i < prefix_stack.size(); ++i)
    {
      if (i > 0) full_key += ".";
      full_key += prefix_stack[i];
    }
    full_key += "." + key;
    return full_key;
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