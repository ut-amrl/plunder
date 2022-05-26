#!/bin/bash
changed_files=$(git diff --name-only);
changed_files+=$(git diff --cached --name-only);

for file in $changed_files; do
    if [[ $file == *"src/"* ]]; then
        if [[ $file == *".cpp" ]] || [[ $file == *".hpp" ]]
        then
            echo "Formatting: $file"
            clang-format -i "$file"
        fi
    fi
done

echo "Format done!"
