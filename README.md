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

### The Result of  ARM experiment

Hardware and OS Info

* Test Server Environment:  Laptop
  * CPU: Intel(R) Core(TM) i5-8300H CPU @ 2.30GHz (8 core)
  * RAM: 16GB RAM
  * OS: Ubuntu 18.04 Desktop (x86_64).
* Test Client Environment: Raspberry 4B
  * CPU: ARM Cortex-A72 @ 1.50GHz (4 core)
  * RAM: 2GB RAM
  * OS: Ubuntu 20.04 Server (aarch64).

Each case repeats 5 times, mean and population standard deviation of each metrics show: 

| case                                                    | time(s)        | cpu(%)          | memory(%)      | rss(MB)         | pss(MB)         | uss(MB)         |
| ------------------------------------------------------- | -------------- | --------------- | -------------- | --------------- | --------------- | --------------- |
| normal                                                  | 17.9995±0.0297 | 154.5693±4.3300 | 12.1529±0.0134 | 224.6066±0.2482 | 116.2888±0.2616 | 101.0638±0.2317 |
| dynamic composition with  component_container_isolated  | 18.0296±0.0098 | 125.0175±4.6471 | 4.3974±0.0120  | 81.2709±0.2212  | 78.1058±0.0929  | 76.2697±0.1015  |
| dynamic composition with  component_container_isolated2 | 18.0335±0.0040 | 108.9113±2.8575 | 4.3704±0.0068  | 80.7735±0.1258  | 78.0515±0.1732  | 76.1973±0.1997  |
| dynamic composition with  component_container_mt        | 18.0111±0.0082 | 145.3000±7.9405 | 4.2420±0.0201  | 78.3999±0.3706  | 74.9570±0.3226  | 73.1115±0.3322  |
| manual composition                                      | 18.0219±0.0070 | 110.9000±1.4405 | 4.2655±0.0054  | 78.8340±0.1001  | 76.8103±0.1392  | 74.6381±0.1479  |

* **dynamic composition with component_container_isolated** (**default composition in Nav2**) saves 19.11% CPU and 32.83% memory (**pss**) over normal non-composed.
* **dynamic composition with component_container_isolated2** acquires similar result as **manual composition**, it saves 29.53% CPU and 32.88% memory (**pss**) over normal non-composed. 

