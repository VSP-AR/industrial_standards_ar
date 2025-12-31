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
