This document is a declaration of software quality for the `zenoh_cpp_vendor` package, based on the guidelines in [REP-2004](https://github.com/ros-infrastructure/rep/blob/rep-2004/rep-2004.rst).


# zenoh_cpp_vendor Quality Declaration

The package `zenoh_cpp_vendor` claims to be in the **Quality Level 2** category.

Below are the rationales, notes, and caveats for this claim, organized by each requirement listed in the [Package Requirements for Quality Level 1 in REP-2004](https://www.ros.org/reps/rep-2004.html).

## Version Policy [1]

### Version Scheme [1.i]

`zenoh_cpp_vendor` uses `semver` according to the recommendation for ROS Core packages in the [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#versioning).

### Version Stability [1.ii]

`zenoh_cpp_vendor` is at or above a stable version, i.e. `>= 1.0.0`.

### Public API Declaration [1.iii]

This is a vendor package for `zenoh-c` and `zenoh-cpp` and as such does not declare its own API.

### API Stability Policy [1.iv]/[1.vi]

`zenoh_cpp_vendor` does not have an API of its own, and will limit its dependency on `zenoh-c` and `zenoh-cpp` to API stable releases within a released ROS distribution.

### ABI Stability Policy [1.v]/[1.vi]

`zenoh_cpp_vendor` does not have an ABI of its own, and will limit its dependency on `zenoh-c` and `zenoh-cpp` to ABI stable releases within a released ROS distribution.

## Change Control Process [2]

`zenoh_cpp_vendor` follows the recommended guidelines for ROS Core packages in the [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#change-control-process).

### Contributor Origin [2.ii]

This package uses DCO as its confirmation of contributor origin policy. More information can be found in [CONTRIBUTING](../CONTRIBUTING.md).

### Peer Review Policy [2.iii]

All pull requests will be peer-reviewed, check the [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#change-control-process) for additional information.

### Continuous Integration [2.iv]

All pull requests must pass CI on all [tier 1 platforms](https://www.ros.org/reps/rep-2000.html#support-tiers).

### Documentation Policy [2.v]

All pull requests must resolve related documentation changes before merging.

## Documentation [3]

### Feature Documentation [3.i]

`zenoh_cpp_vendor` does not have features other than importing the external dependency `zenoh-c` and `zenoh-cpp` and therefore does not require feature documentation.

### Public API Documentation [3.ii]

`zenoh_cpp_vendor` does not have an API and therefore does not require API documentation.

### License [3.iii]

The license for `zenoh_cpp_vendor` is Apache 2.0, and a summary is in each source file, the type is declared in the [`package.xml`](package.xml) manifest file, and a full copy of the license is in the [`LICENSE`](../LICENSE) file. The vendored library, `zenoh-c` and `zenoh-cpp` license is Apache 2.0 as stated in its [quality declaration](zenoh_q_declaration.md) document (Section 5.iii).

### Copyright Statements [3.iv]

The copyright holders each provide a statement of copyright in each source code file in `zenoh_cpp_vendor`.

There is an automated test which runs a linter that ensures each file has at least one copyright statement. Latest linter result report can be seen [here](https://ci.ros2.org/view/nightly/job/nightly_linux-aarch64_release/lastBuild/testReport/zenoh_cpp_vendor/copyright/).

## Testing [4]

`zenoh_cpp_vendor` is a package providing solely CMake files and therefore does not require API or feature tests and has no coverage or performance requirements (sections [4.i - 4.iv]).

### Feature Testing [4.i]

Each feature in `rmw_zenoh_cpp` has corresponding tests which simulate typical usage, and they are located in different ROS 2 packages.
New features are required to have tests before being added.

### Public API Testing [4.ii]

Each part of the public API has tests, and new additions or changes to the public API require tests before being added.
The tests aim to cover both typical usage and corner cases, but are quantified by contributing to code coverage.

### Coverage [4.iii]

Lastest code coverage can be found [here](https://ci.ros2.org/job/ci_linux_coverage/lastSuccessfulBuild/cobertura/).

### Performance [4.iv]

`rmw_zenoh_cpp` does not currently have performance tests.

### Linters and Static Analysis [4.v]

`zenoh_cpp_vendor` uses and passes all the ROS 2 standard linters and static analysis tools as described in the [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#linters-and-static-analysis). Passing implies there are no linter/static errors when testing against CI of supported platforms.

## Dependencies [5]

### Direct and Optional Runtime ROS Dependencies [5.i]/[5.ii]

`zenoh_cpp_vendor` does not have direct/optional runtime ROS dependencies.

### Direct Runtime non-ROS Dependency [5.iii]

`zenoh_cpp_vendor` depends directly on the external dependency `zenoh-c` and `zenoh-cpp`, which is qualified as quality level x in its [Quality Declaration](./zenoh_q_declaration.md).

## Platform Support [6]

`zenoh_cpp_vendor` supports all of the tier 1 platforms as described in [REP-2000](https://www.ros.org/reps/rep-2000.html#support-tiers).

## Security [7]

### Vulnerability Disclosure Policy [7.i]

This package conforms to the Vulnerability Disclosure Policy in [REP-2006](https://www.ros.org/reps/rep-2006.html).

# Current status Summary

The chart below compares the requirements in the REP-2004 with the current state of the libyaml package.
|Number|  Requirement| Current state |
|--|--|--|
|1| **Version policy** |---|
|1.i|Version Policy available | ✓ |
|1.ii|Stable version |✓|
|1.iii|Declared public API|✓|
|1.iv|API stability policy|✓|
|1.v|ABI stability policy|✓|
|1.vi_|API/ABI stable within ros distribution|✓|
|2| **Change control process** |---|
|2.i| All changes occur on change request | ✓|
|2.ii| Contributor origin (DCO, CLA, etc) | ✓|
|2.iii| Peer review policy | ✓ |
|2.iv| CI policy for change requests | ✓ |
|2.v| Documentation policy for change requests | ✓ |
|3| **Documentation** | --- |
|3.i| Per feature documentation | ✓ |
|3.ii| Per public API item documentation | ✓ |
|3.iii| Declared License(s) | ✓ |
|3.iv| Copyright in source files| ✓ |
|3.v.a| Quality declaration linked to README | ✓ |
|3.v.b| Centralized declaration available for peer review |✓|
|4| Testing | --- |
|4.i| Feature items tests | ✓ |
|4.ii| Public API tests | ✓ |
|4.iii.a| Using coverage |✓ |
|4.iii.a| Coverage policy | ✓ |
|4.iv.a| Performance tests (if applicable) | ☓ |
|4.iv.b| Performance tests policy| ✓ |
|4.v.a| Code style enforcement (linters)| ✓ |
|4.v.b| Use of static analysis tools | ✓ |
|5| Dependencies | --- |
|5.i| Must not have ROS lower level dependencies | ✓ |
|5.ii| Optional ROS lower level dependencies| ✓ |
|5.iii| Justifies quality use of non-ROS dependencies |✓|
|6| Platform support | --- |
|6.i| Support targets Tier1 ROS platforms| ✓ |
|7| Security | --- |
|7.i| Vulnerability Disclosure Policy | ✓ |

Comparing this table with the [Quality Level Comparison Chart of REP2004](https://github.com/ros-infrastructure/rep/blob/master/rep-2004.rst#quality-level-comparison-chart) lead us to decide that this package qualifies as Quality Level 2.
