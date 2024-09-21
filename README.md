## Make a workspace and clone the repository
`mkdir -p mcsim_ws/src ; cd mcsim_ws/src`

Clone with ssh: `git clone git@github.com:Andeshog/mcsimpython_with_ros2.git`

or clone with https: `git clone https://github.com/Andeshog/mcsimpython_with_ros2.git`

## Build and source
`cd ~/mcsim_ws ; colcon build ; source install/setup.bash`

## Activate virtual environment
`python3 -m venv venv`

`source venv/bin/activate`

`pip install MCSimPython`


## Run a demo
`ros2 run mcsim_odom_demo odom_demo.py`

`ros2 run mcsim_odom_demo odom_demo_with_waves.py`
