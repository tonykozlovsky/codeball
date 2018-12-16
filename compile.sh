set -ex

if [ "$1" != "base" ]; then
    rm MyStrategy.cpp
    cp -rn /src/* ./
    cp /src/MyStrategy.h ./ || true
fi

find -type f -name "*.cpp" | xargs g++ -std=c++17 -static -fno-optimize-sibling-calls -fno-strict-aliasing -D_LINUX -lm -s -x c++ -O2 -Wall -Wtype-limits -Wno-unknown-pragmas -o /output/MyStrategy