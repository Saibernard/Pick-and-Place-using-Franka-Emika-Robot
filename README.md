# Pick-and-Place-using-Franka-Emika-Robot
Pick and Place using Franka Emika 6 DOF robot

# Project: PANDA Arm Block Manipulation with Geometric Inverse Kinematics

## Scope

This project aims to leverage concepts from the course to enable a 7-DOF PANDA arm to pick static and dynamic blocks and place them in a desired goal position using a Geometric Based Inverse Kinematics (IK) solver. The project involves testing in simulation and hardware to ensure performance under varying conditions.

## Method

Two approaches were formulated - one for static blocks and another for dynamic blocks. Geometric IK solver with a 6-DOF constraint was utilized. A custom planner for static blocks and an advanced version for dynamic blocks were developed to prevent collisions. The robot's movement was optimized, and joint actuation sequences were refined.

## Static Approach

A sequential strategy was devised to grasp static blocks efficiently. Blocks were detected using the camera, and a custom planner ensured safe and precise movement to reach the blocks. Tuning IK input matrix for block height adjustment was essential. A tapping motion was incorporated to orient the blocks before grasping. The robot ensured the block's grip before proceeding.

## Dynamic Approach

Dynamic blocks were approached using an advanced planner with a funneling technique to avoid collisions and maximize efficiency. Hyper-parameter tuning of the IK input matrix ensured accurate block retrieval. A waiting period allowed the dynamic block to align with the end effector before grasping.

## Strategy Evaluation

Multiple strategies were tested in simulation and hardware. Analysis of block stacking strategies helped optimize point accumulation. Different joint actuation sequences were compared for efficiency.

## Lessons Learned

Optimal joint actuation sequence was crucial to avoid collisions. Camera orientation played a role in block detection. Challenges like light reflection were addressed by modifying the robot's approach. Adjusting strategy based on the opponent's performance was advantageous.

## Conclusion

The project successfully implemented Geometric IK solving, enabling the PANDA arm to pick and place blocks accurately. Rigorous testing and strategy refinement led to confident competition performance. Challenges highlighted the importance of adapting to real-world conditions. Overall, the project showcased the practical application of course concepts and algorithmic strategies in a dynamic scenario.

