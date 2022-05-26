#!/usr/bin/env sh
export OMP_NUM_THREADS=12
./bin/ldips-l3 -ex_file examples/nice/merged.json -lib_file ops/social_ref.json -feat_depth 2 -window_size 3 -debug
mv synthd/dipsl3 synthd/nice
mkdir synthd/dipsl3/
./bin/ldips-l3 -ex_file examples/greedy/merged.json -lib_file ops/social_ref.json -feat_depth 2 -window_size 3 -debug
mv synthd/dipsl3 synthd/greedy
