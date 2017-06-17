#!/bin/bash

set -x

./build/mask_fusion ./data/masks

if which meshlab &>/dev/null; then
  meshlab out_mask_fusion.ply
else
  echo "Please install meshlab to view mesh file 'out_mask_fusion.ply'"
  echo "  sudo apt-get install meshlab"
fi

set +x
