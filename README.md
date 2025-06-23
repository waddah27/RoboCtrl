
This platform is created for sake of trajectory planning and motion control of robotics and autonomous machines. It is on going porejct and this readme will be 
updated each time a new feature/ functionality is added.



1. Build the project:
```bash
mkdir -p build && cd build/
cmake ..
make
```

2. To run c++ standalone code, in `build/` directory do the following:
```bash
$ ~/RoboCtrl/build$ ./run 

TODO -- Control system initialize
Received command - Position: 1, Velocity: 10
Trajectory planned to 1 with max velocity 10
plannerTrapz.x_t = 0
planner.trajectory->x_d = 1
planner.trajectory->x_t = 0
planner.trajectory->v_max = 10
current pose trapezND: 0 0 0 
current pose trapezND: 0.0334077 0.0668153 0.100223 
current pose trapezND: 0.133631 0.267261 0.400892 
current pose trapezND: 0.267261 0.534522 0.801784 
current pose trapezND: 0.400892 0.801784 1.20268 
current pose trapezND: 0.534522 1.06904 1.60357 
current pose trapezND: 0.668153 1.33631 2.00446 
current pose trapezND: 0.801784 1.60357 2.40535 
current pose trapezND: 0.875288 1.75058 2.62586 
current pose trapezND: 0.943218 1.88644 2.82965 
current pose trapezND: 1 2 3 
plannerTrapzND is done! 
current pose trapezND: 1 2 3 
```

3. To build the python `roboctrl` package, go to `python/` and run
```bash
$ python -m venv venv_name --system-site-packages
$ source venv_name/bin/activate
$ pip install -e . 

```  
and wait until the packages are being built properly.
4. To verify if the python-c++ bining works fine, please run the `python/test.py` script it should give u the following output:
```bash
$ ~/RoboCtrl/python$ python test.py 

Trajectory planned to 10 with max velocity 2
0.1
['__name__', '__doc__', '__package__', '__loader__', '__spec__', 'TrajectoryState', 'LinearStepPlanner', 'TrapezoidalPlanner', 'TrapezoidalPlannerND', '__file__']

```
For mathematical basis of what we are doing here, please review `docs/`
