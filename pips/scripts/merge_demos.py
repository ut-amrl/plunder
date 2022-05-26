import json
import sys

if (len(sys.argv) != 3):
  print(len(sys.argv))
  print("Expects 2 arguments: paths to input files to combine")
  raise SystemExit

with open(sys.argv[1]) as f:
    data1 = json.load(f)

with open(sys.argv[2]) as f:
    data2 = json.load(f)

data = data1 + data2

with open('merged.json', 'w') as json_file:
    json_file.write(json.dumps(data, indent=4, sort_keys=True))
