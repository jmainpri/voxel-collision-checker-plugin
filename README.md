voxel-collision-checker-plugin
==============================

Dependencies:

* OpenRave : http://openrave.org/docs/latest_stable/


Install:

    mkdir build
    cd build
    cmake ..
    make install
    
Set up your enviroment:

    export OPENRAVE_PLUGINS=$HOME/voxel-collision-checker-plugin/plugins:$OPENRAVE_PLUGINS
    
Run:
    
    cd examples
    python simple_example.py
