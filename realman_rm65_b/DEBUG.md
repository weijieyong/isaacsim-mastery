
get the following error if didnt specify `body_cylinders` in `realman_rm65_b\rmpflow\rm65_rmpflow_common.yaml`
```
2025-05-23 01:50:35 [40,866ms] [Error] [omni.kit.app._impl] [py stderr]: Traceback (most recent call last):
2025-05-23 01:50:35 [40,866ms] [Error] [omni.kit.app._impl] [py stderr]:   File "/isaac-sim/my_examples/realman_rm65_b/custom.py", line 265, in <module>
2025-05-23 01:50:35 [40,866ms] [Error] [omni.kit.app._impl] [py stderr]:     main()
2025-05-23 01:50:35 [40,866ms] [Error] [omni.kit.app._impl] [py stderr]:   File "/isaac-sim/my_examples/realman_rm65_b/custom.py", line 212, in main
2025-05-23 01:50:35 [40,866ms] [Error] [omni.kit.app._impl] [py stderr]:     rmpflow, my_controller = setup_motion_controller(my_realman, config_paths)
2025-05-23 01:50:35 [40,866ms] [Error] [omni.kit.app._impl] [py stderr]:   File "/isaac-sim/my_examples/realman_rm65_b/custom.py", line 131, in setup_motion_controller
2025-05-23 01:50:35 [40,867ms] [Error] [omni.kit.app._impl] [py stderr]:     rmpflow = mg.lula.motion_policies.RmpFlow(
2025-05-23 01:50:35 [40,867ms] [Error] [omni.kit.app._impl] [py stderr]:   File "/isaac-sim/exts/isaacsim.robot_motion.motion_generation/isaacsim/robot_motion/motion_generation/lula/motion_policies.py", line 81, in __init__
2025-05-23 01:50:35 [40,867ms] [Error] [omni.kit.app._impl] [py stderr]:     self._policy = lula.create_rmpflow(rmpflow_config)
2025-05-23 01:50:35 [40,867ms] [Error] [omni.kit.app._impl] [py stderr]: RuntimeError: [Lula] /lula/src/lula/rmpflow/interface/rmpflow.cpp:427: error: Failed check (body_capsules.IsSequence()): 
```

same goes for `body_collision_controllers` 

```
2025-05-23 01:52:30 [32,714ms] [Error] [omni.kit.app._impl] [py stderr]: Traceback (most recent call last):
2025-05-23 01:52:30 [32,714ms] [Error] [omni.kit.app._impl] [py stderr]:   File "/isaac-sim/my_examples/realman_rm65_b/custom.py", line 265, in <module>
2025-05-23 01:52:30 [32,715ms] [Error] [omni.kit.app._impl] [py stderr]:     main()
2025-05-23 01:52:30 [32,715ms] [Error] [omni.kit.app._impl] [py stderr]:   File "/isaac-sim/my_examples/realman_rm65_b/custom.py", line 212, in main
2025-05-23 01:52:30 [32,715ms] [Error] [omni.kit.app._impl] [py stderr]:     rmpflow, my_controller = setup_motion_controller(my_realman, config_paths)
2025-05-23 01:52:30 [32,715ms] [Error] [omni.kit.app._impl] [py stderr]:   File "/isaac-sim/my_examples/realman_rm65_b/custom.py", line 131, in setup_motion_controller
2025-05-23 01:52:30 [32,715ms] [Error] [omni.kit.app._impl] [py stderr]:     rmpflow = mg.lula.motion_policies.RmpFlow(
2025-05-23 01:52:30 [32,715ms] [Error] [omni.kit.app._impl] [py stderr]:   File "/isaac-sim/exts/isaacsim.robot_motion.motion_generation/isaacsim/robot_motion/motion_generation/lula/motion_policies.py", line 81, in __init__
2025-05-23 01:52:30 [32,715ms] [Error] [omni.kit.app._impl] [py stderr]:     self._policy = lula.create_rmpflow(rmpflow_config)
2025-05-23 01:52:30 [32,715ms] [Error] [omni.kit.app._impl] [py stderr]: RuntimeError: [Lula] /lula/src/lula/rmpflow/interface/rmpflow.cpp:390: error: Failed check (body_collision_controllers.IsSequence()): 
```

currently set to arbitrary value first