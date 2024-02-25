# Arena-CrowdSim
**Arena-CrowdSim**  is an extension of [pedsim_ros](https://github.com/Arena-Rosnav/pedsim_ros) that adds highly-configurable interfaces for modeling social state machines and social force models.

![crowdsim_dataflow](https://github.com/Arena-Rosnav/crowdsim/assets/50558925/555e5934-8168-4f1b-9eb1-95c18f55cb02)


## Usage
### Arena-Rosnav
The easiest way to practically test CrowdSim is to use it within the [Arena-Rosnav](https://github.com/Arena-Rosnav/arena-rosnav) framework. Install Arena-Rosnav and launch a CrowdSim simulation using
```sh
roslaunch arena_bringup start_arena.launch entity_manager:=crowdsim
```

### Standalone
CrowdSim can be launched as a standalone node using
```sh
roslaunch crowdsim crowdsim.launch
```

with optional launch arguments
| Argument | Description |
| --- | --- |
| sfm | name of social force model to use |
| scene_file | scene file for pedsim_ros |
| visualize | start pedsim visualization |
