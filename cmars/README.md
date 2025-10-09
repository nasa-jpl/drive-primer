# Build Instructions
- Compile Chrono (Last tested on this [commit](https://github.com/uwsbel/chrono-wisc/commit/9474f5c941ea86e2b10f1c9a8bc56ec4d02346ab))
    - Compile with Vehicle, Parser, FSI
    - Initialize with submodules, run `./buildURDF.sh` in `{your_chrono_dir}/contrib`
        - This is a slightly out of date script so it takes some effort, be warned
        - I will try to update this
        - Take note of where the packages are installed, you will need to point to the cmake directories when compiling both Chrono and this project
    - Requires minimum CUDA 12.0, gcc 11.2, eigen 3.4.0

- Once Chrono is compiled, it is a typical CMake build process 