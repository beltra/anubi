# Navigation Profiles

This context defines Nav2 profile language for tuning and experiments.
It exists so profile selection stays explicit and repeatable across sim and real wrappers.

## Language

**Navigation Profile**:
A named Nav2 parameter set selected by launch or wrapper arguments.
_Avoid_: Nav mode

**Default Safe Profile**:
The standard STVL-backed profile for normal operation.
_Avoid_: Default config

**Narrow-Gap Profile**:
An STVL-backed profile tuned for tighter passage experiments.
_Avoid_: Tight mode

**Forward-Static Profile**:
A comparison profile using a more constrained forward-biased navigation setup.
_Avoid_: Static mode

**Route Experiment**:
An optional route_server run that leaves the normal safety chain unchanged.
_Avoid_: Route mode

**Benchmark Scenario**:
A repeatable simulation case used to compare profile behavior.
_Avoid_: Test world

## Relationships

- A **Navigation Profile** selects one Nav2 YAML unless an explicit params file overrides it.
- The **Default Safe Profile** is the normal wrapper default.
- A **Narrow-Gap Profile** and **Forward-Static Profile** are opt-in comparison profiles.
- A **Route Experiment** adds route_server without changing the command chain.
- A **Benchmark Scenario** measures a **Navigation Profile** without becoming a runtime default.

## Example Dialogue

> **Dev:** "Can we make the benchmark profile the default?"
> **Domain expert:** "Only after it becomes the **Default Safe Profile** deliberately; otherwise it remains a **Benchmark Scenario** or opt-in **Navigation Profile**."

## Flagged Ambiguities

- "mode" is overloaded between SLAM mode, wrapper mode, and Nav2 profile; use **Navigation Profile** for Nav2 YAML selection.
