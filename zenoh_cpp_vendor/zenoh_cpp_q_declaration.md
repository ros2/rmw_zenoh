# External Dependency Quality declaration for zenoh-cpp

This document is a declaration of software quality for the `zenoh-cpp` external dependency, based on the guidelines in [REP-2004](https://github.com/ros-infrastructure/rep/blob/rep-2004/rep-2004.rst).

The [zenoh-cpp](https://github.com/eclipse-zenoh/zenoh-cpp) external dependency is a C++ binding based on the [zenoh-c](https://github.com/eclipse-zenoh/zenoh-c) C binding, itself based on the main [Zenoh implementation written in Rust](https://github.com/eclipse-zenoh/zenoh).
It is maintained by [Eclipse Zenoh GitHub organization](https://github.com/eclipse-zenoh) together with other repositories.
First, a summary discussing how this library is qualified is presented, and then it will be listed how this library matches the standards defined for ROS packages.

## Summary

[Zenoh](https://zenoh.io/) is an open source communication protocol and middleware designed to facilitate efficient data distribution across heterogeneous systems. It provides location-transparent abstractions for high performance pub/sub and distributed queries.
In 2023, an investigation and survey were carried out within the ROS community to identify [an into alternative middleware solutions](https://discourse.ros.org/t/investigation-into-alternative-middleware-solutions/32642) for ROS 2. The [final report](https://discourse.ros.org/uploads/short-url/o9ihvSjCwB8LkzRklpKdeesRTDi.pdf) concluded that Zenoh best meets the requirements, and will be chosen as an alternative middleware. Zenoh was also the most-recommended alternative by users [in the survey](https://docs.google.com/forms/d/1GWb7RrSPkvdgl49LMrsTyoAy3i29LO6AuFSIUzsuwrs/viewanalytics).

The [core of Zenoh](https://github.com/eclipse-zenoh/zenoh) is developped in Rust, ensuring performances, reliability and safety with regard to memory usage: .
[`zenoh-c`](https://github.com/eclipse-zenoh/zenoh-c) provides a C binding based on the Rust core of Zenoh. [`zenoh-cpp`](https://github.com/eclipse-zenoh/zenoh-cpp) provides a C++ binding based on `zenoh-c`. Both are designed as minimal wrappers around the Zenoh API and types, with most of the code and quality assurance efforts concentrated in Zenoh Rust.

All are supported for the majority of the OS platforms, as shown in the 1.2.1 release assets for [zenoh](https://github.com/eclipse-zenoh/zenoh/releases/tag/1.2.1) and [zenoh-c](https://github.com/eclipse-zenoh/zenoh-c/releases/tag/1.2.1) (`zenoh-cpp` consists only in header files):

- Linux with glibc for x86_64, aarch64, armv7 and armv6
- Linux with musl for x86_64 and aarch64
- Windows with MSVC or MinGW for x86_64
- MacOS for x86_64 and aarch64

All meets the basic requirements for a software platform in terms of testing its basic functionality, providing a valid license for the code used and a public GitHub repository with the changes made to the code over time.

Considering the previously mentioned reasons, we consider this library to be robust and reliable and at Quality Level 2.

# Comparison with ROS packages quality standards

## Version policy [1]

### Version Scheme [1.i]

All Zenoh projects (including `zenoh-cpp, zenoh-c` and `zenoh`) adhere to [semver 2.0.0](https://semver.org/spec/v2.0.0.html):
The version numbers are organised as `MAJOR.MINOR.PATCH where all three components are non-negative decimal numbers. Version number policy llows the following rules:

- MAJOR version is incremented when an incompatible API or protocol change is made;
- MINOR version when functionality is added in a backwards compatible. MINOR is source compatible. The project strives to also maintain nary compatibility and protocol interoperability. Any breaking change on those aspects will be documented in the Zenoh [CHANGELOG](https://thub.com/eclipse-zenoh/zenoh/releases).;
- PATCH version when backwards compatible bug fixes are made. PATCH is binary compatible and has interoperable protocol.

See also the [Releases section](https://www.eclipse.org/projects/handbook/#release) of the Eclipse project handbook which discusses Major, Minor, and Service release criteria (where Service corresponds to PATCH in the above description).

### Version Stability [1.ii]

All Zenoh projects are stable since `1.0.0`. The latest releases and all history of changes can be found here:

- [zenoh-cpp](https://github.com/eclipse-zenoh/zenoh-cpp/releases)
- [zenoh-c](https://github.com/eclipse-zenoh/zenoh-c/releases)
- [zenoh](https://github.com/eclipse-zenoh/zenoh/releases)

### Public API Declaration [1.iii]

As a C++ library, elements available in [zenoh.hxx](https://github.com/eclipse-zenoh/zenoh-cpp/blob/main/include/zenoh.hxx) are considered to be the library's public API of `zenoh-cpp`.

### API Stability Policy [1.iv]

The Public API of `zenoh-cpp` include some unstable elements which are all enclosed in conditional groups with `defined_FEATURE_UNSTABLE_API)` as condition.

For the Public API not declared as unstable:

- `zenoh-cpp` provides stability for PATCH and MINOR releases.
- `zenoh-cpp` strives to provide stability for MAJOR releases.
- `zenoh-cpp` does not guarantee stability for MAJOR releases.

For the Public API declared as unstable:

- `zenoh-cpp` won't change in PATCH releases.
- `zenoh-cpp` may change for MINOR and MAJOR releases.

### ABI Stability Policy [1.v]

`zenoh-cpp` only consists in header files which are compiled within the `rmw_zenoh_cpp` library. Those files are headers are calling C functions from `zenoh-c` which is built as a shared library, with `zenoh` Rust code statically linked in this library. Hence, the ABI Stability concerns are for `zenoh-c`.

`zenoh-c` provides ABI stability for PATCH releases and strives to provide ABI stability for MINOR releases, for all the Public API (stable or unstable). `zenoh-c` does not guarantee ABI stability for MINOR or MAJOR releases.

### ABI and ABI Stability Within a Released ROS Distribution [1.vi]

ROS 2 users do not interact directly with `zenoh-cpp`, `zenoh-c` or `zenoh` Rust. The mapping of the RMW interface to the `zenoh-cpp` APIs provided by the `rmw_zenoh_cpp` package hides any API or ABI changes from ROS 2 users.

Based on the ABI Stability Policy, `zenoh-c` PATCH releases can be accepted as updates within a Released ROS Distribution. MINOR releases can likely be accepted as updates after assessing the binary compatibility. `zenoh-c` MINOR and MAJOR releases can, at least in principle, be accepted as updates within a Released ROS Distribution if the update is accompanied by an update of the `rmw_zenoh_cpp` package.

## Change Control Process [2]

All Zenoh projects follow the recommended guidelines of the [Eclipse Development Process](https://www.eclipse.org/projects/dev_process/).

The Eclipse Foundation manages write access to project repositories, allowing only designated [Committers](https://www.eclipse.org/projects/dev_process/#4_1_Committers),
who have been voted for in elections overseen by the Eclipse Foundation, to commit changes.

API and ABI stability is part of the review process. The Eclipse Zenoh project runs CI and tests.
The Eclipse Zenoh project runs ROS CI for changes that are likely to affect ROS.

### Change Requests [2.i]

All changes occur through pull requests made in their respective repositories:

- [zenoh-cpp pull requests](https://github.com/eclipse-zenoh/zenoh-cpp/pulls)
- [zenoh-c pull requests](https://github.com/eclipse-zenoh/zenoh-c/pulls)
- [zenoh pull requests](https://github.com/eclipse-zenoh/zenoh/pulls)

### Contributor Origin [2.ii]

All Zenoh projects use DCO as its confirmation of contributor origin policy.
More information can be found in [Eclipse Foundation's DCO policy](https://www.eclipse.org/legal/DCO.php).
Eclipse projects furthermore require that an [Eclipse Contributor Agreement](https://www.eclipse.org/legal/ECA.php) is on file with the Eclipse Foundation.

### Peer Review Policy [2.iii]

All pull requests must pass peer-review except when no-one is able to provide a review for a PR introduced by a Committer.
The exception exists solely as an escape hatch if no review can be obtained within a reasonable time frame while there is an urgent need for the change introduced by the PR.
Check [Eclipse Developer Process](https://www.eclipse.org/projects/dev_process/) for additional information.

### Continuous Integration [2.iv]

Pull requests are required to pass all tests in the CI system prior to merging, unless Committers consider there is sufficient evidence
that a failure is the result of a mishap unrelated to the change.
All CI results are public and cover x64 platforms running Linux, macOS and Windows:

- [zenoh-cpp CI results](https://github.com/eclipse-zenoh/zenoh-cpp/actions)
- [zenoh-c CI results](https://github.com/eclipse-zenoh/zenoh-c/actions)
- [zenoh CI results](https://github.com/eclipse-zenoh/zenoh/actions)

Moreover, `zenoh-c` is compiled in the ROS 2 buildfarm for all [tier 1 platforms](https://www.ros.org/reps/rep-2000.html#support-tiers).

Currently nightly results can be seen here:
* [linux-aarch64_release](https://ci.ros2.org/view/nightly/job/nightly_linux-aarch64_release/lastBuild/testReport/zenoh_cpp_vendor/)
* [linux_release](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/testReport/zenoh_cpp_vendor/)
* [mac_osx_release](https://ci.ros2.org/view/nightly/job/nightly_osx_release/lastBuild/testReport/zenoh_cpp_vendor/)
* [windows_release](https://ci.ros2.org/view/nightly/job/nightly_win_rel/lastBuild/testReport/zenoh_cpp_vendor/)

### Documentation Policy [2.v]

All pull requests must resolve related documentation changes before merging.

## Documentation [3]

### Feature Documentation [3.i]

Provided for the whole project on [zenoh.io](https://zenoh.io)

### Public API Documentation [3.ii]

Provided documentations for each binding:

- [zenoh-cpp documentation](https://zenoh-cpp.readthedocs.io/en)
- [zenoh-c documentation](https://zenoh-c.readthedocs.io/en)
- [zenoh documentation](https://docs.rs/zenoh/latest/zenoh/)

### License [3.iii]

All Zenoh APIs are made available under the terms of the Eclipse Public License 2.0, or the Apache License, Version 2.0.
Each repository include a `NOTICE.md` with links to more information about these licenses, and a `LICENSE` file with the full terms.

- `zenoh-cpp`: [NOTICE.md](https://github.com/eclipse-zenoh/zenoh-cpp/blob/main/NOTICE.md#declared-project-licenses)
  and [LICENSE](https://github.com/eclipse-zenoh/zenoh-cpp/blob/main/LICENSE).
- `zenoh-c`: [NOTICE.md](https://github.com/eclipse-zenoh/zenoh-c/blob/main/NOTICE.md#declared-project-licenses)
  and [LICENSE](https://github.com/eclipse-zenoh/zenoh-c/blob/main/LICENSE).
- `zenoh`: [NOTICE.md](https://github.com/eclipse-zenoh/zenoh/blob/main/NOTICE.md#declared-project-licenses)
  and [LICENSE](https://github.com/eclipse-zenoh/zenoh/blob/main/LICENSE).

### Copyright Statements [3.iv]

Each Zenoh repository includes in its `NOTICE.md` file a policy regarding content copyright:

- `zenoh-cpp`: [NOTICE.md](https://github.com/eclipse-zenoh/zenoh-cpp/blob/main/NOTICE.md#copyright)
- `zenoh-c`: [NOTICE.md](https://github.com/eclipse-zenoh/zenoh-c/blob/main/NOTICE.md#copyright)
- `zenoh`: [NOTICE.md](https://github.com/eclipse-zenoh/zenoh/blob/main/NOTICE.md#copyright)

In those repositories, each of the source files containing code include a copyright statement with the license information in the file's header.

### Quality Declaration [3.v]

This document represents the Quality Declaration document for the `zenoh-cpp` ROS dependency and the depending Zenoh projects: `zenoh-c` and `zenoh`.

## Testing [4]

### Feature Testing [4.i]

All Zenoh projects provide tests to cover the expected usage of the libraries:

- [`zenoh-cpp`](https://github.com/eclipse-zenoh/zenoh-cpp/tree/main/tests)
- [`zenoh-c`](https://github.com/eclipse-zenoh/zenoh-c/tree/main/tests)
- `zenoh`: following Rust practices, the tests are located in the source code, in functions marked with the `#[test]` attribute, or in a `tests` subdirectory.

In addition of this, the Eclipse Zenoh project conducts integration tests based on the examples of API usages provided for each binding:

- [zenoh-cpp examples](https://github.com/eclipse-zenoh/zenoh-cpp/tree/main/examples)
- [zenoh-c examples](https://github.com/eclipse-zenoh/zenoh-c/tree/main/examples)
- [zenoh examples](https://github.com/eclipse-zenoh/zenoh/tree/main/examples)

### Public API Testing [4.ii]

The aforementioned tests are testing the Public APIs, especially those based on Zenoh examples.

### Coverage [4.iii]

Zenoh publishes [test coverage](https://app.codecov.io/gh/eclipse-zenoh/zenoh) using [Codecov](https://codecov.io) for the Rust core of Zenoh.
New changes are required to include tests coverage. Line coverage is approximately 71% (as of 2025-03-11).

### Performance [4.iv]

While there are no public automated tests or results, there is evidence in PRs that performance does get taken into account (see, e.g., https://github.com/eclipse-zenoh/zenoh/pull/1627).

ZettaScale Technology is running nightly performance tests as part of quality testing on internal servers.

### Linters and Static Analysis [4.v]

`zenoh` in Rust runs [rust-clippy](https://github.com/rust-lang/rust-clippy) for linting and static analysis in its CI.
`zenoh-c` and `zenoh-cpp` run [ClangFormat](https://clang.llvm.org/docs/ClangFormat.html) for linting in their CI.

## Dependencies [5]

### Direct Runtime ROS Dependencies [5.i]

As external dependencies, there are no ROS dependencies in `zenoh-cpp`, `zenoh-c` and `zenoh`.

### Optional Direct Runtime ROS Dependencies [5.ii]

As external dependencies, there are no ROS dependencies in `zenoh-cpp`, `zenoh-c` and `zenoh`.

### Direct Runtime non-ROS Dependency [5.iii]

`zenoh-cpp`, `zenoh-c` and `zenoh` don't require any dependency at runtime.

## Platform Support [6]

This library does not state support for any specific platform, but it is built in the ROS 2 buildfarm for all tier 1 platforms.

## Security [7]

### Vulnerability Disclosure Policy [7.i]

All Zenoh projects conform to the Vulnerability Disclosure Policy in REP-2006. The Eclipse Project Handbook states the project's [vulnerability disclosure policy](https://www.eclipse.org/projects/handbook/#vulnerability-disclosure) in detail.
