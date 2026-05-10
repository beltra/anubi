# Workspace Architecture

This context defines the package ownership language for the Anubi ROS 2 workspace.
It exists so feature work lands in the right layer instead of coupling robot-specific adapters to robot-agnostic autonomy.

## Language

**Platform Adapter**:
A robot-specific package that exposes hardware behavior through the shared autonomy contracts.
_Avoid_: Hardware app, robot code

**Autonomy Package**:
A robot-agnostic package that owns navigation, localization, safety, recovery, or LIO behavior.
_Avoid_: Platform package, robot driver

**Core Interface**:
A shared ROS message or service contract consumed across packages.
_Avoid_: Helper message, package-local type

**Simulation Package**:
A package that provides Gazebo worlds, robot models, sensor models, or scenario runtime tools.
_Avoid_: Test-only package

**Vendor Package**:
Vendored third-party code kept in-tree with minimal local edits.
_Avoid_: Core package

## Relationships

- A **Platform Adapter** consumes **Core Interfaces** and publishes shared autonomy topics.
- An **Autonomy Package** must not depend on Go2-only implementation details when a shared topic or interface exists.
- A **Simulation Package** mirrors shared sim-real topics even when its internal Gazebo transport differs.
- A **Vendor Package** is wrapped by an **Autonomy Package** before it participates in the stack contract.

## Example Dialogue

> **Dev:** "Should recovery live in `go2_bringup` because Go2 needs it?"
> **Domain expert:** "No. The recovery behavior is an **Autonomy Package** concern; `go2_bringup` only wires the Go2 adapter into the shared navigation contract."

## Flagged Ambiguities

- "robot code" can mean either **Platform Adapter** or **Autonomy Package**; use the narrower term.
