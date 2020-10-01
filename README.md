# `tonioviz` to the rescue

## Prerequisites

For the moment, make sure you install:
- Pangolin
- GTSAM

and let me know if it breaks...

Then just checkout the `examples/SimpleCheckout.cpp` exec and let me know if it
runs for you.

<!-- You can install the necessary dependencies by just running the -->
<!-- `install-dependencies.sh` script. This will install: -->

<!-- - Pangolin -->

<!-- In addition, you need to have the following: -->

<!-- - GTSAM (negociable if you guys think we should remove this dependency, should -->
<!--   be pretty easy to do) -->

## Examples

Here are some examples:

- GTSAM-based:

- Eigen-based:

- ROS-based:

## Troubleshooting

### `SimpleCheckout.cpp` data

Simple checkout was created with Kurran's Blue ROV data in mind, which you can
find in this
[link](https://drive.google.com/drive/folders/1c-FjAgZI91IUzn-MGl9Tl4CUMox31dPq).

### Filesystem

If you get an error regarding `filesystem` due to

```cpp
#include <filesystem>
```

and

```cpp
std::filesystem::exists(name)
```

from `/src/DataUtils.cpp`, in the `LoadImages` function, then you need to
upgrade your GCC compilers. An easy peasy way to do it is by following
[this](https://linuxize.com/post/how-to-install-gcc-compiler-on-ubuntu-18-04/)
guide.
