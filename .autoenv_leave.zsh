#!/usr/bin/env zsh

leave () {
  export PATH=$_PATH
  export LD_LIBRARY_PATH=$_LD_LIBRARY_PATH
  export CMAKE_PREFIX_PATH=$_CMAKE_PREFIX_PATH
}

leave
