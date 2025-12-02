Robot Description
===================

This guide explains how to set up a ROS2 robot description package using the Universal Robots (UR) description packages as a reference. 
Robot descriptions are essential in ROS2 because they define the structure, geometry, kinematics, dynamics, and 
visualization details of a robot. They serve as the foundation for simulation, motion planning, visualization in RViz, and integration with 
`ros2_control` for hardware drivers. A well-structured description package ensures compatibility with industrial standards, enabling robots to 
be used across different simulators, controllers, and applications without modification.


Setting up the ROS2 Robot Description package
---------------------------------------------
The best practice is to create a new directory for every new workspace.

.. code-block:: console

    mkdir -p ~/<directory_name>/src
    cd ~/<directory_name>/src

Keep the packages in your workspace into the ``src`` directory. The above code creates a ``src`` directory inside ``<directory_name>`` and navigates into it.

.. note:: text
    Ensure you are still inside ``/<directory_name>/src`` before following the next steps.

Use the following command to generate a new package:


.. code-block:: console

        ros2 pkg create --build-type ament_cmake <package_name>

If the `<package_name>` is for example named as ``my_robot_description``, then the command creates the directory structure and essential files:

.. code-block:: text

        my_robot_description/ 
        ├── CMakeLists.txt 
        ├── package.xml 
        ├── include/my_robot_description/ 
        └── src/

- ``ros2 pkg create`` - The Ros2 CLI tool for creating a new package with the necessary files and structures
- ``--build-type ament_cmake``- sets up the package to be built using CMake, managed by ament, which is ROS 2's build toolchain.

     - You use this build type for description packages, C++ nodes, or launch/configuration-based packages.

     - If your package only contains Python code, use ``--build-type ament_python`` instead.


Adding Dependencies
~~~~~~~~~~~~~~~~~~~~

After creating the package, the next step is to add the required dependencies in the ``package.xml`` file.
Dependencies ensure that all necessary tools and libraries are available when building or running the package.

Open the ``package.xml`` file located inside your package directory and add the following lines under the existing tags:

.. code-block:: xml
    
    <buildtool_depend>ament_cmake</buildtool_depend>
    <!-- Runtime dependencies -->
    <exec_depend>xacro</exec_depend>
    <exec_depend>robot_state_publisher</exec_depend>
    <exec_depend>joint_state_publisher_gui</exec_depend>
    <exec_depend>rviz2</exec_depend>

These dependencies serve the following purposes:

``ament_cmake``
 - Specifies that the package uses the ``ament_cmake`` build system, required for building and installing files.

``xacro``
 - Enables the use of .xacro (XML Macros) files, which allow modular and reusable robot description files.
 - Commonly used to simplify URDFs by including parameters, macros, and reusable components.

``robot_state_publisher``
 - Reads the robot's URDF and publishes the corresponding TF (transform) tree, which defines how the links and joints are connected in space.
 - This is essential for visualizing the robot in RViz or running simulations.

``joint_state_publisher_gui``
 - Provides a GUI tool to adjust joint values interactively, allowing visualization of the robot's movement in RViz.
 - Useful when the robot is not yet connected to real hardware.

``rviz2``
 - Required when launching RViz from your package to visualize the robot description directly.

Creating the URDF Folder
~~~~~~~~~~~~~~~~~~~~~~~~~
Once the dependencies are set up, the next step is to create the folder structure for the robot description package.
A well-organized structure ensures that URDF, meshes, and launch files are easy to locate, maintain, and use across different tools and simulators.

Navigate to your package directory and create the following folders:

.. code-block:: console

    cd ~/<directory_name>/src/<package_name>
    mkdir -p urdf/inc meshes/visual meshes/collision launch rviz config

This creates the standard directory structure used by most robot description packages:

.. code-block:: text

    <package_name>/
    ├── urdf/ 
    │   ├── inc/
    ├── meshes/
    │   ├── visual/
    │   └── collision/
    ├── launch/
    ├── rviz/
    └── config/

- urdf/
     Contains the robot's URDF (Unified Robot Description Format) or Xacro files.
     These describe the robot's structure — links, joints, geometry, and kinematics.

  - inc/
      Stores smaller modular Xacro files such as links.xacro, joints.xacro, or materials.xacro.
      These files are included into the main URDF/Xacro file using <xacro:include>.
      This modular design improves maintainability for complex robot models. 
- meshes/
     Stores 3D models of the robot's parts used for visualization and collision detection.

      - visual/ 
         High-detail meshes used for visualization in RViz or Gazebo.
      - collision/ 
         Simplified meshes used for efficient collision checking in motion planning.    
- launch/
     Contains launch files (usually written in Python) to load and visualize the robot model in RViz or simulators.

- rviz/
     Stores RViz configuration files (e.g., camera view, TF tree, display settings) for quick visualization setup.

- config/
     Holds configuration files used by other tools or nodes.
     
         Commonly used for:
             - ros2_control settings
             - Controller YAML files
             - Joint limits and transmission definitions
             - Other parameters for simulation or hardware integration
  
Updating the CMakeLists.txt
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To ensure these directories are installed and accessible after building, modify the ``CMakeLists.txt`` file:

.. code-block:: console

    cmake_minimum_required(VERSION 3.5)
    project(<package_name>)

    find_package(ament_cmake REQUIRED)

    # Install resource directories

    install(
    DIRECTORY urdf meshes launch rviz config
    DESTINATION share/${PROJECT_NAME}
    )

    ament_package()

This ensures that when the package is built, all necessary files (URDFs, meshes, configs, etc.) are copied into the install and made  discoverable by launch files or other packages.

Build and verify installation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Once the file is updated, return to the root of your workspace and build and source the package:

.. code-block:: console

    colcon build
    source install/setup.bash

Now, verify that all files were installed correctly:

.. code-block:: console

    ls install/<package_name>/share/<package_name>

Xacro 
~~~~~~

A **Xacro** (XML Macros) file is an extended form of URDF that supports macros, variables, and mathematical operations, making robot descriptions modular and easier to maintain.  

The full explanation of Xacro syntax and advanced constructs, you can reference the ROS tutorial for:
`Using Xacro to clean up your code <https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html>`_

When building modular robot descriptions in ROS 2, it is a **best practice** to separate your robot model into **two main Xacro files**:

1. **A main entry file** → `<robot_name>.urdf.xacro`  
2. **A macro definition file** → `<robot_name>_macro.xacro`

This separation helps keep your model **organized**, **reusable**, and **scalable** for different robot configurations.

**1. Purpose of each file**


.. table:: Purpose of Each File
    :align: center

    +----------------------------+--------------------+--------------------------------------------------+---------------------------------------------+
    | File                       | Role               | Contains                                         | Used For                                    |
    +============================+====================+==================================================+=============================================+
    | `<robot_name>_macro.xacro` | Library file       | Defines robot structure in a ``<xacro:macro>``   | Template; not run directly.                 |
    +----------------------------+--------------------+--------------------------------------------------+---------------------------------------------+
    | `<robot_name>.urdf.xacro`  | Main entry point   | Includes the macro and instantiates it with      | Entry file to generate URDF for RViz or     |
    |                            |                    | parameters (YAML files, constants, etc.)         | simulation.                                 |
    +----------------------------+--------------------+--------------------------------------------------+---------------------------------------------+

**2.  Why use Two files**

Using two files provides several benefits:

-  **Modularity** - The robot structure is defined once (in the macro) and reused for multiple variants. 
-  **Reusability** - The same macro can be used with different parameters (link lengths, joint limits, visuals).  
-  **Clarity** - The main `.urdf.xacro` file remains short and readable; it only handles includes and arguments.  
-  **Parameterization** - You can pass different YAML configuration files for different robots without duplicating the entire URDF.
