[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

# Manipulability Metrics

<p align="center">
  <img width="640" src="https://user-images.githubusercontent.com/678580/96242832-3731dd00-0fa4-11eb-990d-495cb4b44640.png">
</p>


This repository provides a library to easily compute multiple manipulability metrics for a single or dual-arm manipulator.

The list of metrics that can be computed for a single-arm manipulator are:

- **Inverse condition number:**
Computes the quotient between the smaller and the greater singular values of the Jacobian.
The inverse of the condition number is used so that a bigger value represents better manipulability, which is the case with the rest of the metrics.
- **Manipulability Measure:**
Computes the square root of the determinant of the product of the Jacobian and the Jacobian transpose. It's equivalent to the product of all singular values of the Jacobian.
- **Minimum Singular Value:**
Returns the value of the minimum singular value.
- **Task-Oriented Manipulability Measure (TOMM):**
Computes how well the manipulability ellipsoid of the manipulator matches the provided _desired_ manipulability ellipsoid.

In addition to the above metrics that can be used for an individual arm in a dual-arm setup, the following dual-arm-specific metrics are available:

- **Dual-Arm Manipulability Measure (DAMM)<sup>[1](#damm-note)</sup>:**
Approximates the combined manipulability ellipsoid of both manipulators as the intersection of the manipulability matrices of each manipulator and returns a value proportional to its volume.
- **Task-Oriented Dual-Arm Manipulability Measure (TODAMM):**
Computes how well the combined manipulability ellipsoid of both manipulator matches the provided _desired_ manipulability ellipsoid.

For more information about the TOMM, DAMM and TODAMM metrics, refer to: S. Lee, "Dual redundant arm configuration optimization with task-oriented dual arm manipulability," in IEEE Transactions on Robotics and Automation, vol. 5, no. 1, pp. 78-97, Feb. 1989, doi: 10.1109/70.88020. ([link](https://ieeexplore.ieee.org/document/88020)).

## Installation/setup

This library is provided by the `manipulability_metrics` ROS package.
The accompanying `manipulability_metrics_examples` package contains two usage examples, for the single and dual-arm use-cases.

Both packages require C++14 to be built, and therefore only support ROS Melodic or newer distros.

This packages can be built and used following the usual ROS workflow:
- Add the sources of this repository to a ROS workspace
- Use `rosdep` to install any missing dependency
- Build with your prefered tool: `catkin_tools` (tested), `catkin_make` or `catkin_make_isolated`.

Any package in the same or overlying workspaces can then use the library in the usual way.

In order to run the examples, issue any of the two commands below after having built the packages and sourced the workspace's setup script:
- `roslaunch manipulability_metrics_examples panda_manipulability_demo.launch`
- `roslaunch manipulability_metrics_examples dual_panda_manipulability_demo.launch`

## Usage

The following snippet illustrates the simplest possible use case.

```cpp
#include <manipulability_metrics/manipulability_metrics.h>
#include <iostream>

int main(int argc, char** argv)
{
  const auto model = getModel();  // Parse or build a urdf::ModelInterface instance

  const auto chain = manipulability_metrics::Chain{ model, "chain_root", "chain_tip" };

  const auto jntarray = getJointPositions();  // Fill KDL::JntArray with joint positions

  std::cout << "Inverse Condition Number: " << manipulability_metrics::inverseConditionNumber(chain, jntarray) << '\n';

  return 0;
}
```

More detailed use cases can be found in the examples package.

## Acknowledgements

This development is supported by the European Union's Horizon 2020 project RobotUnion.
This project has received funding from the European Union's Horizon 2020 research and innovation programme under grant agreement No 779967.

The opinions and arguments expressed reflect only the author's view and reflect in no way the European Commission's opinions. The European Commission is not responsible for any use that may be made of the information it contains.

[![RobotUnion logo](https://user-images.githubusercontent.com/678580/96243105-92fc6600-0fa4-11eb-87ee-0e41ae040c55.png)](https://robotunion.eu/)

---

<a name="damm-note">[1]</a>:
the paper proposes methods to compute the DAMM for two manipulators in two configurations: _tight cooperation_, where no freedom of motion exists between both arms, e.g. when carrying a bulky load bi-manually; and _loose cooperation_, where partial relative motion between both arms is permitted, e.g. when assembling parts held by each arm.
However, only the _tight cooperation_ scenario is supported by this library at the moment.
