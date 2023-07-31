taming transport for point clouds

It's a bazel project. See BUILD file for targets. 

Run `bazel run :encdec --cxxopt=-std=c++17` for seeing encoder and decoder in action. 

You may need to run:

You need to increase OS UDP Buffer sizes. On Linux, run:

sudo sysctl -w net.core.rmem_default=26214400

sudo sysctl -w net.core.rmem_max=26214400