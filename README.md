
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
$ ./run
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
