#!/usr/bin/env zsh

setup() {
  export _LD_LIBRARY_PATH=$LD_LIBRARY_PATH
  export _CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH
  export _PATH=$PATH

  export CMAKE_PREFIX_PATH=$_autoenv_this_dir/devel
  export LD_LIBRARY_PATH=$_autoenv_this_dir/devel/lib
  export PATH=$_autoenv_this_dir/devel/bin:$_autoenv_this_dir/build:$PATH
}

_autoenv_this_dir=${0:a:h}
setup