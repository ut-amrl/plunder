#!/bin/bash

header_files="$(find $(dirname $(realpath $0))/../src/ -name \*.hpp)"
source_files="$(find $(dirname $(realpath $0))/../src/ -name \*.cpp)"

echo "Formatting Headers!"

for file in $header_files; do
    echo "Formatting: $file"
    clang-format -style=Google -i "$file"
done

echo "Formatting Source!"

for file in $source_files; do
    echo "Formatting: $file"
    clang-format -style=Google -i "$file"
done

echo "Format done!"
