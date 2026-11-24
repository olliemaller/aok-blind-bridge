// Extra names the web page answers to, such as a router's DNS name (aok-blinds.lan).
// Header-only and free of Arduino so the host tests can check it.
#pragma once

#include <cctype>
#include <cstddef>
#include <string>

namespace aok {

constexpr int kMaxExtraHosts = 4;

// One DNS name: dot-separated labels of letters, digits and hyphens, 1-63 characters
// each and not starting or ending with a hyphen, up to 253 characters in all.
inline bool hostNameValid(const std::string& s) {
  if (s.empty() || s.size() > 253) return false;
  size_t label = 0;
  for (size_t k = 0; k < s.size(); ++k) {
    const unsigned char c = static_cast<unsigned char>(s[k]);
    if (c == '.') {
      if (label == 0 || s[k - 1] == '-') return false;
      label = 0;
      continue;
    }
    if (!std::isalnum(c) && c != '-') return false;
    if (c == '-' && label == 0) return false;
    if (++label > 63) return false;
  }
  return label > 0 && s.back() != '-';
}

// Checks a list separated by commas or spaces and writes it to `out` in lower case,
// comma-separated, without duplicates. Returns how many names, or -1 when a name is
// invalid or there are more than kMaxExtraHosts.
inline int parseHostList(const std::string& in, std::string& out) {
  out.clear();
  int count = 0;
  std::string name;
  for (size_t k = 0; k <= in.size(); ++k) {
    const char c = k < in.size() ? in[k] : ',';
    if (c != ',' && c != ' ' && c != '\t') {
      name += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
      continue;
    }
    if (name.empty()) continue;
    if (name.back() == '.') name.pop_back();  // a fully qualified name's trailing dot
    if (!hostNameValid(name)) return -1;
    if (("," + out + ",").find("," + name + ",") == std::string::npos) {
      if (++count > kMaxExtraHosts) return -1;
      if (!out.empty()) out += ',';
      out += name;
    }
    name.clear();
  }
  return count;
}

// Whether `host` (lower case, no port) is one of the names in a list from parseHostList.
inline bool hostInList(const std::string& host, const std::string& list) {
  return !host.empty() && !list.empty() && ("," + list + ",").find("," + host + ",") != std::string::npos;
}

}  // namespace aok
