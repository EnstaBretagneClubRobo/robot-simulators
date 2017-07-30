# Robot simulators

This repository contains classes to simulate the behaviour of:
- car model
- sailboat model (based on L. Jaulin equations)

These simulators can be used to test your control method.  
They are compatible with ROS (see [ros_node_example_car.py](examples/ros_node_example_car.py))  
You can also display their behaviour with matplotlib (see [display_matplotlib_car.py](examples/display_matplotlib_car.py)). **Warning**: it's not in real-time.

## How to use the examples

In order to execute the examples correctly from a terminal, you need to add the repository path to your `PYTHONPATH` before executing the script:

```bash
export PYTHONPATH=$PYTHONPATH:/path/to/repo/
python script.py
```

## TODO:

- [ ] Add PID control example