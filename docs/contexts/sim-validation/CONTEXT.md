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

## Relationships

- A **Benchmark Suite** uses **Benchmark Scenarios** from the navigation-profile context.
- A **Safety Regression** should run through the same command and safety chain as normal bringup.
- A **Scenario Manager** depends on a **World Service Bridge** when Gazebo services are not already exposed.
- A **Validation Report** supports tuning decisions but does not define runtime behavior.

## Example Dialogue

> **Dev:** "Should the scenario manager start by default with sim?"
> **Domain expert:** "No. It supports a **Benchmark Suite** or **Safety Regression** and should remain opt-in unless a validation entrypoint asks for it."

## Flagged Ambiguities

- "test" can mean unit test, launch smoke, benchmark, or safety regression; use **Benchmark Suite** or **Safety Regression** when discussing sim validation.
