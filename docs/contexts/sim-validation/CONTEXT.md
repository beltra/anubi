# Simulation Validation

This context defines simulation benchmark and regression language from the validation branches.
It exists to separate repeatable validation tools from normal robot bringup.

## Language

**Benchmark Suite**:
A repeatable simulation run that records navigation metrics across scenarios.
_Avoid_: Demo run

**Safety Regression**:
A simulation scenario that checks whether safety behavior still stops or avoids unsafe motion.
_Avoid_: Safety test when the scope is unclear

**Scenario Manager**:
The simulation tool that loads obstacle and world scenarios through Gazebo services.
_Avoid_: Spawner

**World Service Bridge**:
The ROS-Gazebo bridge exposing world create, remove, or pose services to scenario tools.
_Avoid_: Gazebo bridge

**Validation Report**:
A machine-readable result artifact from a benchmark or regression run.
_Avoid_: Log

**Offline Bag Replay**:
A validation run that replays a recorded LiDAR (and optional IMU) bag through the same perception, SLAM, and Nav2 costmap chain used on the real robot. It is offline because there is no body odometry, no hardware bridge, and no command chain.
_Avoid_: Bag test, dataset replay

**Replay TF Owner Override**:
The narrow exception that allows DLIO to own `odom -> base_link` during an **Offline Bag Replay** because the EKF lacks **Body Odometry** input. It applies only to bag replay; live runs still follow the lio-fusion **TF Owner** rule.
_Avoid_: DLIO TF mode, offline TF hack

## Relationships

- A **Benchmark Suite** uses **Benchmark Scenarios** from the navigation-profile context.
- A **Safety Regression** should run through the same command and safety chain as normal bringup.
- A **Scenario Manager** depends on a **World Service Bridge** when Gazebo services are not already exposed.
- A **Validation Report** supports tuning decisions but does not define runtime behavior.
- An **Offline Bag Replay** validates the **Corrected LiDAR Cloud** chain, slam_toolbox, and Nav2 costmaps without the safety/command chain. It is the only place a **Replay TF Owner Override** is permitted.

## Example Dialogue

> **Dev:** "Should the scenario manager start by default with sim?"
> **Domain expert:** "No. It supports a **Benchmark Suite** or **Safety Regression** and should remain opt-in unless a validation entrypoint asks for it."

> **Dev:** "Can I just replay the unilidar bag through the real-robot launch?"
> **Domain expert:** "No. That brings up the hardware bridge and the safety/command chain. Use an **Offline Bag Replay** entrypoint that wires only perception, SLAM, and costmaps and applies the **Replay TF Owner Override**."

## Flagged Ambiguities

- "test" can mean unit test, launch smoke, benchmark, or safety regression; use **Benchmark Suite** or **Safety Regression** when discussing sim validation.
- "replay" can mean **Benchmark Suite** rerun, **Safety Regression** rerun, or **Offline Bag Replay**; use the precise term when scope is unclear.
