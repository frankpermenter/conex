#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace {

enum class Access { kUnknown, kPublic, kPrivate, kProtected };

std::string Trim(const std::string& s) {
  const auto first = s.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return "";
  const auto last = s.find_last_not_of(" \t\r\n");
  return s.substr(first, last - first + 1);
}

void AssertNoPublicDoHooks(const std::string& path) {
  std::ifstream in(path);
  ASSERT_TRUE(in.is_open()) << "Failed to open " << path;

  Access access = Access::kUnknown;
  std::string line;
  int line_no = 0;
  while (std::getline(in, line)) {
    ++line_no;
    const std::string t = Trim(line);
    if (t == "public:") access = Access::kPublic;
    if (t == "private:") access = Access::kPrivate;
    if (t == "protected:") access = Access::kProtected;

    const bool looks_like_do_hook =
        t.rfind("void do_", 0) == 0 || t.rfind("bool do_", 0) == 0 ||
        t.rfind("int do_", 0) == 0 || t.rfind("CONEX_STATUS do_", 0) == 0 ||
        t.rfind("virtual void do_", 0) == 0 ||
        t.rfind("virtual bool do_", 0) == 0 ||
        t.rfind("virtual int do_", 0) == 0 ||
        t.rfind("virtual CONEX_STATUS do_", 0) == 0;

    if (looks_like_do_hook) {
      EXPECT_NE(access, Access::kPublic)
          << path << ":" << line_no
          << " exposes do_* hook in public section: " << t;
    }
  }
}

TEST(ConstraintVisibilityTest, DoHooksAreNotPublic) {
  const std::vector<std::string> headers = {
      "conex/constraint.h",           "conex/linear_constraint.h",
      "conex/soc_constraint.h",       "conex/quadratic_cone_constraint.h",
      "conex/equality_constraint.h",  "conex/psd_constraint.h",
      "conex/dense_lmi_constraint.h", "conex/hermitian_psd.h",
  };

  for (const auto& path : headers) {
    AssertNoPublicDoHooks(path);
  }
}

}  // namespace
