# Open IHM
>[!NOTE]
>you need qtcreator and qtcreator-ros to follow this tutorial (see [here](../README.md#qt-creator-and-qt-creator-ros) to install)

- Open qtcreator-ros
- go to File -> Open project or file

![openproject](../conception/img/openWorkspace.png)
- Open the file 'workspace.user'

# Build and run IHM

Open a terminal in the IHM directory and run this command

```
colcon build
source install/setup.bash
./build/robot_mobile_pkg_cpp/my_node_ihm
# if you change the name of project change this line like this ./build/<node_name>/<executable_name>
```
