# Voxblox Ground Truth
Create ground truth voxblox TSDF maps from
* Gazebo worlds
* Ply mesh files

## Install
Make sure that [voxblox](https://github.com/ethz-asl/voxblox#table-of-contents) and [gazebo](http://gazebosim.org/tutorials?tut=ros_installing) are installed, then run
```bash
cd ~/catkin_ws
catkin build voxblox_ground_truth
source devel/setup.bash
```

## Gazebo plugin
#### Demo
Start the demo by running
```bash
roslaunch voxblox_ground_truth gazebo_plugin_demo.launch
```
Then wait for Gazebo and Rviz finish loading. Once they're ready, call
```bash
rosservice call /gazebo/save_voxblox_ground_truth_to_file "file_path: '$HOME/voxblox_ground_truth_demo.tsdf'"
```

The ground truth TSDF map will now be available in your home folder as `~/voxblox_ground_truth_demo.tsdf`.

#### Your own world
In order to use the plugin, it must be loaded as part of your Gazebo world.

To do this, add the following line to your `.world` file right after the `<world name='default'>` tag:
```xml
<plugin name="voxblox_ground_truth_plugin" filename="libvoxblox_ground_truth_plugin.so"/>
```

For an example, see the provided [sample.world](https://github.com/ethz-asl/voxblox_ground_truth/blob/8f868dc4290ebaffa8b4c6435491f3cfa386783d/sample_data/gazebo/worlds/burning_building_rubble.world#L4-L5).

Once your world is ready, launch your simulation in the same way you normally would.

Next, set the desired voxel size with
```bash
rosparam set /voxblox_ground_truth/voxel_size 0.05
```

And finally generate the ground truth map with
```bash
rosservice call /gazebo/save_voxblox_ground_truth_to_file "file_path: '$HOME/your_ground_truth_map.tsdf'"
```

## Ply conversion script
#### Demo
Try the demo with
```bash
roslaunch voxblox_ground_truth ply_importer_demo.launch
```

The ground truth TSDF map will now be available in your home folder as `~/bun_zipper.tsdf`.

#### Your own Ply file
To convert your own `.ply` file, run
```bash
roscore &
rosrun voxblox_ground_truth ply_to_sdf \
       [path_to_mesh_file.ply] [tsdf_output_file_path] [voxel_size] \
       [scale_factor] [X] [Y] [Z] [Qx] [Qy] [Qz] [Qw]
```
Note that the paths must be absolute.

The mesh can be scaled up or down using the `scale_factor`, translated by `X Y Z` and rotated with `Qx Qy Qz Qw`. Elements `Qx Qy Qz` and `Qw` should be set to the imaginary and real parts of a normalized quaternion. No rotation corresponds to `0 0 0 1`. A convenient rotation conversion tool is available on Andre Gaschler's [website](https://www.andre-gaschler.com/rotationconverter/).

In case you'd like to have fun with some random `.ply` meshes, you could download samples from Stanford's 3D Scanning Repository
 [here](http://graphics.stanford.edu/data/3Dscanrep/).