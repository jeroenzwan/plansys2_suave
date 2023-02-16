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
