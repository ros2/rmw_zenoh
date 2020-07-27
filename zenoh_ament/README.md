# zenoh_ament
Ament wrapper package for the zenoh.io FFI

Wraps the headers and compiled static library for the [`rust-master` branch of the `zenoh-net` middleware interface](https://github.com/atolab/eclipse-zenoh/tree/rust-master).



## Warning

Currently the only way to update the headers and static library object are to manually compile the zenoh-net source and copy the relevant files over.

If Zenoh continues to be a promising middleware for basing another RMW implementation on, a script will be written to automate this.