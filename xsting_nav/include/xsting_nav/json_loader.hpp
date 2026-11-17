#ifndef XSTING_NAV__JSON_LOADER_HPP_
#define XSTING_NAV__JSON_LOADER_HPP_

#include <cctype>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace xsting_nav
{

/// Minimal self-contained C++ JSON value tree for fast loading of policy & map files.
class JsonValue
{
public:
  enum class Type { Null, Bool, Number, String, Array, Object };

  Type type = Type::Null;
  bool bool_val = false;
  double num_val = 0.0;
  std::string str_val;
  std::vector<JsonValue> arr_val;
  std::map<std::string, JsonValue> obj_val;

  bool contains(const std::string & key) const
  {
    return type == Type::Object && obj_val.find(key) != obj_val.end();
  }

  const JsonValue & at(const std::string & key) const
  {
    if (type != Type::Object) {
      throw std::runtime_error("JSON value is not an object when accessing key: " + key);
    }
    auto it = obj_val.find(key);
    if (it == obj_val.end()) {
      throw std::runtime_error("Missing required JSON key: " + key);
    }
    return it->second;
  }

  const JsonValue & operator[](size_t idx) const
  {
    if (type != Type::Array || idx >= arr_val.size()) {
      throw std::runtime_error("JSON array index out of bounds");
    }
    return arr_val[idx];
  }

  size_t size() const
  {
    return type == Type::Array ? arr_val.size() : (type == Type::Object ? obj_val.size() : 0);
  }

  double as_double() const
  {
    if (type != Type::Number) {
      throw std::runtime_error("Expected JSON number");
    }
    return num_val;
  }

  float as_float() const { return static_cast<float>(as_double()); }
  int as_int() const { return static_cast<int>(as_double()); }

  const std::string & as_string() const
  {
    if (type != Type::String) {
      throw std::runtime_error("Expected JSON string");
    }
    return str_val;
  }

  std::vector<float> as_float_vec() const
  {
    if (type != Type::Array) {
      throw std::runtime_error("Expected JSON array of floats");
    }
    std::vector<float> out;
    out.reserve(arr_val.size());
    for (const auto & item : arr_val) {
      out.push_back(item.as_float());
    }
    return out;
  }

  static JsonValue parse_file(const std::string & path)
  {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
      throw std::runtime_error("Cannot open JSON file: " + path);
    }
    std::ostringstream oss;
    oss << ifs.rdbuf();
    return parse_string(oss.str());
  }

  static JsonValue parse_string(const std::string & src)
  {
    size_t pos = 0;
    JsonValue val = parse_elem(src, pos);
    skip_ws(src, pos);
    return val;
  }

private:
  static void skip_ws(const std::string & s, size_t & p)
  {
    while (p < s.size() && std::isspace(static_cast<unsigned char>(s[p]))) {
      ++p;
    }
  }

  static std::string parse_raw_string(const std::string & s, size_t & p)
  {
    if (p >= s.size() || s[p] != '"') {
      throw std::runtime_error("Expected '\"' in JSON string");
    }
    ++p;
    std::string out;
    while (p < s.size()) {
      char c = s[p++];
      if (c == '"') {
        return out;
      }
      if (c == '\\') {
        if (p >= s.size()) {
          throw std::runtime_error("Unexpected end of string escape");
        }
        char esc = s[p++];
        switch (esc) {
          case '"': out.push_back('"'); break;
          case '\\': out.push_back('\\'); break;
          case '/': out.push_back('/'); break;
          case 'b': out.push_back('\b'); break;
          case 'f': out.push_back('\f'); break;
          case 'n': out.push_back('\n'); break;
          case 'r': out.push_back('\r'); break;
          case 't': out.push_back('\t'); break;
          default: out.push_back(esc); break;
        }
      } else {
        out.push_back(c);
      }
    }
    throw std::runtime_error("Unterminated JSON string");
  }

  static JsonValue parse_elem(const std::string & s, size_t & p)
  {
    skip_ws(s, p);
    if (p >= s.size()) {
      throw std::runtime_error("Unexpected end of JSON input");
    }
    char c = s[p];
    if (c == '{') {
      JsonValue v;
      v.type = Type::Object;
      ++p;
      skip_ws(s, p);
      if (p < s.size() && s[p] == '}') {
        ++p;
        return v;
      }
      while (p < s.size()) {
        skip_ws(s, p);
        std::string key = parse_raw_string(s, p);
        skip_ws(s, p);
        if (p >= s.size() || s[p] != ':') {
          throw std::runtime_error("Expected ':' after object key");
        }
        ++p;
        v.obj_val.emplace(std::move(key), parse_elem(s, p));
        skip_ws(s, p);
        if (p < s.size() && s[p] == ',') {
          ++p;
          continue;
        }
        if (p < s.size() && s[p] == '}') {
          ++p;
          return v;
        }
        throw std::runtime_error("Expected ',' or '}' in JSON object");
      }
    } else if (c == '[') {
      JsonValue v;
      v.type = Type::Array;
      ++p;
      skip_ws(s, p);
      if (p < s.size() && s[p] == ']') {
        ++p;
        return v;
      }
      while (p < s.size()) {
        v.arr_val.push_back(parse_elem(s, p));
        skip_ws(s, p);
        if (p < s.size() && s[p] == ',') {
          ++p;
          continue;
        }
        if (p < s.size() && s[p] == ']') {
          ++p;
          return v;
        }
        throw std::runtime_error("Expected ',' or ']' in JSON array");
      }
    } else if (c == '"') {
      JsonValue v;
      v.type = Type::String;
      v.str_val = parse_raw_string(s, p);
      return v;
    } else if (s.compare(p, 4, "true") == 0) {
      p += 4;
      JsonValue v;
      v.type = Type::Bool;
      v.bool_val = true;
      return v;
    } else if (s.compare(p, 5, "false") == 0) {
      p += 5;
      JsonValue v;
      v.type = Type::Bool;
      v.bool_val = false;
      return v;
    } else if (s.compare(p, 4, "null") == 0) {
      p += 4;
      return JsonValue{};
    } else if (c == '-' || std::isdigit(static_cast<unsigned char>(c))) {
      const char * start = s.c_str() + p;
      char * end = nullptr;
      double num = std::strtod(start, &end);
      if (end == start) {
        throw std::runtime_error("Invalid JSON number");
      }
      p = static_cast<size_t>(end - s.c_str());
      JsonValue v;
      v.type = Type::Number;
      v.num_val = num;
      return v;
    }
    throw std::runtime_error(std::string("Unexpected JSON token: ") + c);
  }
};

}  // namespace xsting_nav

#endif  // XSTING_NAV__JSON_LOADER_HPP_
