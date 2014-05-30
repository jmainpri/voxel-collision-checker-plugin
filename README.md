voxel-collision-checker-plugin
==============================

The collision checker implements a distance propagtion field in a voxel grid. It automaticly computes the collision points (spheres) to match the geometry of the robot. The collision checker can also return a collision potential or minimal distance to the environment. See the example directory for usage.

![Collision points and voxel sign distance field](abbie.png)

### Dependencies:

* OpenRave : http://openrave.org/docs/latest_stable/


#### Install:

    mkdir build && cd build
    cmake ..
    make install
    
 Set up your enviroment:

    export OPENRAVE_PLUGINS=$HOME/voxel-collision-checker-plugin/plugins:$OPENRAVE_PLUGINS
    
#### Run:
    
    cd examples
    python pr2_simple_example.py

