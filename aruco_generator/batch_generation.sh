#!/usr/bin/bash

rm -rf batch_markers
mkdir batch_markers

for n in {1..100}
do
  build/create_marker "marker$n.png" -d=10 -id=$n -ms=1000
done

echo "Finished, totally $n markers generated!"


