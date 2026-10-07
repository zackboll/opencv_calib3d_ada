#!/bin/sh
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$root"
cc=${CC:-cc}; cxx=${CXX:-c++}
mkdir -p obj/profile-tests
"$cc" -std=c11 -Wall -Wextra -Wpedantic -Werror -Icpp \
    tests/cpp/header_test.c -o obj/profile-tests/header_test
"$cxx" -std=c++17 -Wall -Wextra -Wpedantic -Werror -Icpp \
    tests/cpp/profile_test.cpp -o obj/profile-tests/profile_test
obj/profile-tests/header_test
obj/profile-tests/profile_test
"$cxx" -std=c++17 -Wall -Wextra -Wpedantic -Werror -Icpp \
    tests/cpp/homography_profile_test.cpp -o obj/profile-tests/homography_profile_test
obj/profile-tests/homography_profile_test
