#!/bin/sh
# Borrowed and modified from https://gitlab.freedesktop.org/monado/monado

# Formats all the source files in this project

set -e

if [ ! "${CLANGFORMAT}" ]; then
        for fn in clang-format-9 clang-format-8 clang-format-7 clang-format-6.0 clang-format; do
                if command -v $fn > /dev/null; then
                        CLANGFORMAT=$fn
                        break
                fi
        done
fi

if [ ! "${CLANGFORMAT}" ]; then
        echo "clang-format is not setup on this system!" 1>&2
        exit 1
fi

(
        cd $(dirname $0)/..
        find \
                src/ \
                test/ \
                \( -name "*.c" -o -name "*.cpp" -o -name "*.h" -o -name "*.hpp" \) \
                -exec ${CLANGFORMAT} -i -style=file \{\} +
)
