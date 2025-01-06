# Robot Integration Tutorial

This guide walks you through the steps to integrate a new robot into the PyCRAM framework.

## Steps

### 1. Clone Your Robot Description Repository
Ensure all required meshes for your robot are available by cloning your robot description repository:
```bash
# Replace with the actual repository URL
git clone <your_robot_description_repository_url>
```

### 2. Add URDF to Resources
Copy your robot's URDF file into the PyCRAM resources folder:
```bash
cp <path_to_your_robot.urdf> pycram/resources/robots/
```

### 3. Modify the Launch File
Edit `pycram/launch/ik_and_description.launch` and add the following lines, adjusted for your robot:
```xml
<!-- YourRobotName -->
<group if="$(eval robot == '{your_robot_name}')">
    <param name="robot_description"
          textfile="$(find pycram)/resources/robots/{your_robot_name}.urdf"/>
</group>
```
Replace `{your_robot_name}` with the name of your robot.

### 4. Add a Process Module File
Create a process module file named `{your_robot_name}_process_modules.py` in `pycram/src/pycram/process_modules/` with the following content:

```python
from .default_process_modules import DefaultManager

class {your_robot_name}Manager(DefaultManager):
    def __init__(self):
        super().__init__()
        self.robot_name = "{your_robot_name}"
```

### 5. Update `__init__.py`
Edit the `__init__.py` file in the `process_modules` folder and add the following lines:

```python
from .{your_robot_name}_process_modules import {your_robot_name}Manager

{your_robot_name}Manager()
```

### 6. Generate the Robot Description
Navigate to `pycram/demos/pycram_virtual_building_demos/robot_description_generation/` and execute the provided Jupyter Notebook. Follow its step-by-step instructions to generate the robot description.

### 7. Update the Demo File
Edit `pycram/demos/pycram_bullet_world_demo/demo.py` and replace the current `robot_name` (around line 22) with your robot's name:
```python
robot_name = "{your_robot_name}"
```

## Final Note
Once all steps are completed, your robot should be fully integrated into the PyCRAM framework and ready for use in simulations and demos!

