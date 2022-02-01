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

Run test server (`Gazebo simulator`, `nav2_simple_commander` ) in computer

```bash
# cd nav2_composition_experiment/scripts
python test_server.py 
```
Run test client (`navigation2 system`) in another separate computer
```bash
# cd nav2_composition_experiment/scripts
python test_client.py <server-host-ip>
```
* these two computers should be in a same LAN.
* test client communicate with test server by using UDP socket.

## Experiments

Software Info

* ROS version: Rolling
* Navigation version: [building from source](https://github.com/gezp/navigation2/tree/nav2_composition_experiment)

Test cases:

bring up navigation2 system with different way：

* normal non-composed. 
* dynamic composition with different container :  `component_container_mt`  (from `rclcpp_components`)，`component_container_isolated` (from `rclcpp_components`), `component_container_isolated2` (from current package `nav2_composition_experiment`）
* manual composition (from current package `nav2_composition_experiment`）.

> Tip：`component_container_isolated2` uses `spin()` instead of `spin_until_future_complete()` in `component_container_isolated`.

then, navigate from point A to point B by using `nav2_simple_commander` : `[-2, -0.5, 0.0]` ->`[1.4, -1.6, 0.0]`

Test metrics:

* time: navigation time (from publishing goal to reaching goal).
* cpu : sum of every process CPU percentage.
* memory:  sum of every process memory percentage (rss).
* rss, pss, uss : sum of every process memory info (refer to [psutil](https://psutil.readthedocs.io/en/latest/#psutil.Process.memory_full_info)).

> `memory/rss`  is overestimation for multi process system due to shared library.

### The result of ARM experiment

Hardware and OS Info

* Test Server Environment:  Laptop
  * CPU: Intel(R) Core(TM) i5-8300H CPU @ 2.30GHz (8 core)
  * RAM: 16GB RAM
  * OS: Ubuntu 18.04 Desktop (x86_64).
* Test Client Environment: Raspberry 4B
  * CPU: ARM Cortex-A72 @ 1.50GHz (4 core)
  * RAM: 2GB RAM
  * OS: Ubuntu 20.04 Server (aarch64).

Each case repeats 10 times, mean and population standard deviation of each metrics show: 

| case                                                    | time(s)        | cpu(%)          | memory(%)      | rss(MB)         | pss(MB)         | uss(MB)         |
| ------------------------------------------------------- | -------------- | --------------- | -------------- | --------------- | --------------- | --------------- |
| normal                                                  | 17.8796±0.3799 | 154.2745±3.9111 | 12.1217±0.0337 | 224.0308±0.6225 | 116.6360±0.4050 | 101.1552±0.5018 |
| dynamic composition with  component_container_isolated  | 18.0251±0.0078 | 121.0781±3.6180 | 4.3825±0.0168  | 80.9956±0.3109  | 78.6358±0.1525  | 76.2538±0.1509  |
| dynamic composition with  component_container_isolated2 | 18.0294±0.0066 | 109.6206±2.4342 | 4.3822±0.0203  | 80.9900±0.3755  | 78.6317±0.1787  | 76.2604±0.1830  |
| dynamic composition with  component_container_mt        | 17.9053±0.3374 | 140.3295±7.0591 | 4.2451±0.0404  | 78.4565±0.7459  | 75.5215±0.7145  | 73.1378±0.7127  |
| manual composition                                      | 17.9095±0.3093 | 110.5054±2.8760 | 4.2515±0.0299  | 78.5757±0.5518  | 77.8480±0.4771  | 76.5393±0.4753  |

* **dynamic composition with component_container_isolated** (**default composition in Nav2**) saves 21.51% CPU and 32.58% memory (**pss**) over normal non-composed.
* **dynamic composition with component_container_isolated2** acquires similar result as **manual composition**, it saves 28.94% CPU and 32.58% memory (**pss**) over normal non-composed. 

### The result of X86_64 experiment

Hardware and OS Info

* Test Environment:  Laptop
  * CPU: Intel(R) Core(TM) i5-8300H CPU @ 2.30GHz (8 core)
  * RAM: 16GB RAM
  * OS: Ubuntu 18.04 Desktop (x86_64).
  * Docker version: 20.10.1
* Test Server and Client run in different docker container at the same host machine

Each case repeats 10 times, mean and population standard deviation of each metrics show: 

| case                                                    | time(s)        | cpu(%)         | memory(%)     | rss(MB)         | pss(MB)         | uss(MB)         |
| ------------------------------------------------------- | -------------- | -------------- | ------------- | --------------- | --------------- | --------------- |
| normal                                                  | 18.7317±0.5486 | 48.6068±3.1570 | 1.5226±0.0043 | 242.0994±0.6818 | 118.8527±0.3602 | 102.0541±0.3527 |
| dynamic composition with  component_container_isolated  | 18.9435±0.0219 | 40.1647±4.8537 | 0.4469±0.0022 | 71.0585±0.3446  | 67.7321±0.1906  | 64.7954±0.1925  |
| dynamic composition with  component_container_isolated2 | 19.0534±0.3355 | 36.0938±4.0020 | 0.4490±0.0022 | 71.3879±0.3457  | 67.6770±0.1495  | 64.7348±0.1490  |
| dynamic composition with  component_container_mt        | 18.8268±0.3348 | 46.2794±2.8031 | 0.4417±0.0023 | 70.2375±0.3704  | 66.7185±0.2717  | 63.7811±0.2647  |
| manual composition                                      | 18.7109±0.4431 | 36.6018±4.2282 | 0.4309±0.0018 | 68.5248±0.2807  | 67.1393±0.1842  | 65.3641±0.1790  |

* **dynamic composition with component_container_isolated** (**default composition in Nav2**) saves 17.36% CPU and 43.01% memory (**pss**) over normal non-composed.
* **dynamic composition with component_container_isolated2** acquires similar result as **manual composition**, it saves 25.74% CPU and 43.05% memory (**pss**) over normal non-composed. 

