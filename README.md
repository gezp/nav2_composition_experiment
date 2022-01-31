# nav2_composition_experiment

# nav2_composition_experiment

test composition performance for [navigation2](https://github.com/ros-planning/navigation2).

## Quick Start
Install ROS and build Navigation2 from source (only support Rolling version)

* https://navigation.ros.org/build_instructions/index.html

> Tip: at least 4GB RAM is required to build Navigation2, if you use Raspberry with 1GB or 2GB RAM, you can enable swap memory.

Install dependencies

```bash
pip install psutil
```

Run Gazebo simulator in computer

```bash
# cd nav2_composition_experiment/scripts
python tb3_simulator_server.py 
```
Run navigation2 and metric programs in another separate computer
```bash
# cd nav2_composition_experiment/scripts
python test.py <gazebo-host-ip>
```
* these two computers should be in a same LAN.

## Experiments

Software Info

* ROS version: Rolling
* Navigation version: [building from source](https://github.com/gezp/navigation2/tree/nav2_composition_experiment)

Test cases:

* bring up navigation2 with normal non-composed, dynamic composition with different container, manual composition.
* navigate from point A to point B by using `nav2_simple_commander` : `[-2, -0.5, 0.0]` ->`[1.4, -1.6, 0.0]`

Test metrics

* time: navigation time (from publishing goal to reaching goal).
* cpu : sum of every process CPU percentage.
* memory:  sum of every process memory percentage (rss).
* rss, pss, vms : sum of every process memory info (refer to [psutil](https://psutil.readthedocs.io/en/latest/#psutil.Process.memory_full_info)).

> `memory/rss`  is overestimation for multi process system due to shared library.

### The Result of  ARM experiment

Hardware and OS Info

* Gazebo Simulator Environment:  Laptop
  * CPU: Intel(R) Core(TM) i5-8300H CPU @ 2.30GHz (8 core)
  * RAM: 16GB RAM
  * OS: Ubuntu 18.04 Desktop (x86_64).
* Navigation2 Environment: Raspberry 4B
  * CPU: ARM Cortex-A72 @ 1.50GHz (4 core)
  * RAM: 2GB RAM
  * OS: Ubuntu 20.04 Server (aarch64).

Each case repeats 5 times, mean and population standard deviation of each metrics show: 

| case                                                   | time (s)       | cpu (%)         | memory (%)     | rss (MB)        | pss (MB)        | vms (MB)          |
| ------------------------------------------------------ | -------------- | --------------- | -------------- | --------------- | --------------- | ----------------- |
| normal non-composed                                    | 17.7000±0.2944 | 149.4944±0.6945 | 12.1783±0.0256 | 225.0761±0.4738 | 117.0557±0.2767 | 7812.8816±28.8304 |
| dynamic composition  with component_container_isolated | 17.6073±0.3542 | 119.5300±1.2714 | 4.3546±0.0302  | 80.4801±0.5586  | 77.9121±0.4675  | 2421.0454±0.1438  |
| dynamic composition  with component_container_mt       | 17.6798±0.1225 | 142.2383±6.5773 | 4.2422±0.0131  | 78.4035±0.2417  | 74.8609±0.2052  | 2365.6924±2.1683  |
| manual composition                                     | 17.7566±0.2630 | 102.9381±1.2838 | 4.2783±0.0404  | 79.0696±0.7458  | 76.5826±0.4608  | 2419.3515±0.0607  |

* dynamic composition composition with component_container_isolated (**default composition in Nav2**) saves 20.04% CPU and 33.44% memory (**pss**) over normal non-composed.

* manual composition performs best in cpu utilization, it saves 31.14% CPU and 34.57% memory (**pss**) over normal non-composed.

