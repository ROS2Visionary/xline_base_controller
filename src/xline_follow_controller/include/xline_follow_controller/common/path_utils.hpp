#pragma once

#include <string>
#include <cstdlib>
#include <cctype>

namespace xline
{
namespace path_utils
{

inline std::string expand_env(const std::string& path)
{
  std::string result;
  result.reserve(path.size());

  for (size_t i = 0; i < path.size(); ++i)
  {
    if (path[i] == '$')
    {
      std::string var;
      size_t j = i + 1;

      // 处理 ${VAR} 形式
      if (j < path.size() && path[j] == '{')
      {
        ++j;
        while (j < path.size() && path[j] != '}')
        {
          var += path[j++];
        }
        i = (j < path.size()) ? j : path.size() - 1;
      }
      else
      {
        // 处理 $VAR 形式
        while (j < path.size()
               && (std::isalnum(static_cast<unsigned char>(path[j])) || path[j] == '_'))
        {
          var += path[j++];
        }
        i = j - 1;
      }

      const char* val = std::getenv(var.c_str());
      if (val)
      {
        result += val;
      }
    }
    else
    {
      result += path[i];
    }
  }

  return result;
}

inline std::string resolve_path(const std::string& raw)
{
  std::string p = expand_env(raw);

  // 绝对路径直接返回
  if (!p.empty() && p[0] == '/')
  {
    return p;
  }

  // 相对路径优先挂到 XLINE_WS_ROOT 下
  const char* ws_root = std::getenv("XLINE_WS_ROOT");
  if (ws_root && *ws_root)
  {
    if (p.empty())
    {
      return std::string(ws_root);
    }
    return std::string(ws_root) + "/" + p;
  }

  // 没有环境变量就按当前行为处理（保持相对路径）
  return p;
}

}  // namespace path_utils
}  // namespace xline
