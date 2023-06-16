workspace(name = "main")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# Setup rules_foreign_cc (CMake integration)
http_archive(
    name = "rules_foreign_cc",
    strip_prefix = "rules_foreign_cc-8d540605805fb69e24c6bf5dc885b0403d74746a", # 0.9.0
    url = "https://github.com/bazelbuild/rules_foreign_cc/archive/8d540605805fb69e24c6bf5dc885b0403d74746a.tar.gz",
)

load("@rules_foreign_cc//foreign_cc:repositories.bzl", "rules_foreign_cc_dependencies")

rules_foreign_cc_dependencies()

http_archive(
    name = "rules_pcl",
    url = "https://github.com/kgreenek/rules_pcl/archive/v1.1.0.tar.gz",
    sha256 = "987dd22ac4637093414ff2b9291d0e9e29f3ef156a12d5471eedaa2d5beb0a93",
    strip_prefix = "rules_pcl-1.1.0",
)

load("@rules_pcl//bzl:repositories.bzl", "pcl_repositories")
pcl_repositories()

# NOTE: This must be loaded after the call to pcl_repositories().
load("@rules_pcl//bzl:init_deps.bzl", "pcl_init_deps")
pcl_init_deps()
