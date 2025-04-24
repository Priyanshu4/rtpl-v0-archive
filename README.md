# RTPL

RTPL (Rapid Tree Planning Library) is a Rust motion planning library which implements RRT and RRT variants (only RRT* right now). 
This is an early version of the library which is not actively maintained. While I may develop an improved version in the future,  there are currently no guarantees of continued development.

## Design
The library is designed in an abstract way such that the same planning algorithms can apply to a variety of robots with different state spaces, dynamics, distance metrics, sampling distributions, collision checking functions, etc.

These are the main components/traits used by the planners in this library:
- `State`: A state in the state-space of your robot
- `Motion`: Describes an edge/motion between two states in the state-space. Each motion must contain its ending state.
  - `HasCost`: Additional trait for motions which have a cost (needed for RRT*)
  - `Discretizable`: Additional trait required by collision checkers which discretize motions
- `SamplingDistribution`: Randomly samples states in your state-space
- `Steering`: Generates motions between two states or from one state in the direction of another
- `ValidityChecker`: Provides functions to check if a state or edge is valid (not in collision).
- `DistanceMetric`: Defines a distance between two states.
- `NearestNeighbors`: Provides functions find the closest neighbors of a state or all states within a radius.
- `Region`: Defines a sets of states and provides the ability to check if a state is contained within the region.
- `TerminationCondition`: Defines a condition that indicates when planning should terminate.

For geometric path planning (i.e. euclidean space, no need for robot dynamics) implementations of these traits are provided in the `real_vector_state` and `real_vector_state::euclidean` modules.

## Examples
The library provides several examples out of the box.
- rrt_2d
  - Runs RRT for geometric path planning in a 2d environment (using 2D RealVectorState and euclidean traits)
  - Uses a sampling distribution with 5% goal bias
  - `cargo run --release --example rrt_2d`
- rrt_star_2d
  - Runs RRT* for geometric path planning in a 2D environment (using 2D RealVectorState and euclidean traits)
  - `cargo run --release --example rrt_star_2d`
- ur3e
  - Runs RRT* on a UR3E manipulator specified by a URDF
  - Rust module writes a json to `examples/ur3e/path.json` which is then read by `pathvis.py` for visualization
    1. `cargo run --release --example ur3e` to plan the path
    2. `uv run examples/pathvis.py examples/ur3e/path.json` to visualize the robot's path with pybullet
        - If you do not have `uv` installed, you can run with `python>=3.11` or higher after installing `pybullet>=3.2.6`
