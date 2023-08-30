# plansys2_suave
PDDL files for plansys2

Step 1:

Download the plansys2 package

## Running the code

In a terminal, run the plansys2 system with the inspect_pl_domain file loaded in:

```bash
ros2 launch plansys2_bringup plansys2_bringup_launch_distributed.py model_file:=path_to_file/inspect_pl_domain.pddl
```

In a new terminal populate the problem_expert:

```bash
ros2 run plansys2_terminal plansys2_terminal
```

In a separate terminal, start up the plansys_terminal:

```bash
ros2 run plansys2_terminal plansys2_terminal
```

In the plansys_terminal, run plan:

```bash
get plan
```

## Running full simulation

Make sure to navigate to the pipeline inspection workspace and source for every new terminal opened.

```bash
source install/setup.sh
```

Start ardusub simulation of UUV:

```bash
sim_vehicle.py -L RATBeach -v ArduSub --model=JSON --console
```

In a new terminal start the gazebo simulation:

```bash
ros2 launch suave simulation.launch.py x:=17.0 y:=2.0
```

Next, in a new terminal launch the action nodes and load in the PDDL file:

```bash
ros2 launch plansys2_suave plansys2_simple_launch.py
```

Afterwards, in a new terminal launch the system modes monitor to gain an insight
into the current system modes being used:

```bash
ros2 launch system_modes mode_monitor.launch.py
modelfile:=src/suave/suave/config/suave_modes.yaml
```

Start mission file in a new terminal:

```bash
ros2 run suave_missions start_mission
```

Run the plan handler node in a new terminal:

```bash
ros2 run mros2_mock plan_handler_node
```

Following, launch system modes with the system modes bridge and all the monitor nodes:

```bash
ros2 launch suave_metacontrol mock_suave_metacontrol.launch.py
```

Finally, launch the mros reasoner:

```bash
ros2 launch suave_metacontrol metacontrol.launch.py
```
