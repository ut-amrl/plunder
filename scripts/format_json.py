#!/usr/bin/env python3

import json
import os

script_directory = os.path.dirname(os.path.realpath(__file__))
example_directory = os.path.join(script_directory, '..', 'examples')
operator_directory = os.path.join(script_directory, '..', 'ops')

all_example_files = [os.path.join(example_directory, x) for x in os.listdir(example_directory)]
all_operator_files = [os.path.join(operator_directory, x) for x in os.listdir(operator_directory)]

for json_file in all_example_files + all_operator_files:
    print('Formatting: ' + json_file)
    with open(json_file, 'r') as f:
        file_contents = f.read()
    parsed_json = json.loads(file_contents)
    pretty_json = json.dumps(parsed_json, indent=2, sort_keys=False)
    with open(json_file, 'w') as f:
        f.write(pretty_json)