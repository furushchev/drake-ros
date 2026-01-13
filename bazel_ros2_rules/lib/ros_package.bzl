# -*- python -*-

load("@bazel_ros2_rules//lib:ament_index.bzl", "AmentIndex")
load("@bazel_skylib//lib:paths.bzl", "paths")

def _ros_package_impl(ctx):
    """
    Implementation of the ros_package rule.

    Creates a ROS package structure in runfiles that can be discovered
    by the ament index, enabling use of launch_ros.actions.Node in launch files.
    """
    # 1. Create package marker in ament index
    package_marker_path = paths.join(
        ctx.attr.prefix,
        "share/ament_index/resource_index/packages/",
        ctx.attr.package_name,
    )
    package_marker_out = ctx.actions.declare_file(package_marker_path)
    ctx.actions.write(
        output = package_marker_out,
        content = "",
    )

    runfiles_symlinks = {package_marker_path: package_marker_out}

    # 2. Symlink package.xml if provided
    if ctx.file.package_xml:
        package_xml_path = paths.join(
            ctx.attr.prefix,
            "share",
            ctx.attr.package_name,
            "package.xml",
        )
        runfiles_symlinks[package_xml_path] = ctx.file.package_xml

    # 3. Symlink launch files to share/<package_name>/launch/
    for launch_file_target in ctx.attr.launch_files:
        for file in launch_file_target.files.to_list():
            # Use the file's basename as the installed name
            launch_path = paths.join(
                ctx.attr.prefix,
                "share",
                ctx.attr.package_name,
                "launch",
                file.basename,
            )
            runfiles_symlinks[launch_path] = file

    # 4. Symlink executables to lib/<package_name>/
    transitive_runfiles = []
    for exec_target, exec_name in ctx.attr.executables.items():
        # Get the actual executable file(s)
        files = exec_target[DefaultInfo].files.to_list()

        # Find the executable file - it's typically the one with execute permission
        # or matches the target name
        exec_file = None
        target_name = exec_target.label.name

        # First try to find a file matching the target name exactly
        for f in files:
            if f.basename == target_name:
                exec_file = f
                break

        # If not found, take the first file that's likely an executable
        # (not a .py file that's a shim, not a manifest file)
        if not exec_file:
            for f in files:
                if not f.path.endswith(".py") and not f.path.endswith(".runfiles_manifest"):
                    exec_file = f
                    break

        if not exec_file:
            fail("Could not find executable file for target '{}'. Files: {}".format(
                exec_target.label,
                [f.path for f in files],
            ))

        # Create symlink at lib/<package_name>/<exec_name>
        exec_path = paths.join(
            ctx.attr.prefix,
            "lib",
            ctx.attr.package_name,
            exec_name,
        )
        runfiles_symlinks[exec_path] = exec_file

        # Include transitive runfiles from executable
        if hasattr(exec_target[DefaultInfo], "default_runfiles"):
            transitive_runfiles.append(exec_target[DefaultInfo].default_runfiles)

    # 5. Merge runfiles
    merged_runfiles = ctx.runfiles(root_symlinks = runfiles_symlinks)
    for rf in transitive_runfiles:
        merged_runfiles = merged_runfiles.merge(rf)

    return [
        AmentIndex(prefix = ctx.attr.prefix),
        DefaultInfo(runfiles = merged_runfiles),
    ]

ros_package = rule(
    attrs = {
        "package_name": attr.string(
            mandatory = True,
            doc = "ROS package name for discovery by launch_ros.actions.Node",
        ),
        "executables": attr.label_keyed_string_dict(
            mandatory = False,
            default = {},
            allow_files = False,
            doc = "Mapping of binary targets to executable names (e.g., {':my_binary': 'my_node'})",
        ),
        "package_xml": attr.label(
            mandatory = False,
            allow_single_file = [".xml"],
            doc = "Optional package.xml file for full ROS 2 compliance",
        ),
        "launch_files": attr.label_list(
            mandatory = False,
            default = [],
            allow_files = [".py", ".xml", ".yaml"],
            doc = "Launch file targets to install in share/<package>/launch/ (e.g., [':my_launch.py'])",
        ),
        "prefix": attr.string(
            default = "ros_package",
            doc = "Runfiles prefix for the ament index",
        ),
    },
    implementation = _ros_package_impl,
    provides = [AmentIndex],
    doc = """
Creates a ROS package structure that can be discovered via the ament index.

This rule enables the use of launch_ros.actions.Node in launch files by
registering executables as part of a ROS package. The package will be
discoverable by name, and executables can be launched using their registered
names.

Example usage:

```python
load("@bazel_ros2_rules//lib:ros_package.bzl", "ros_package")

ros_package(
    name = "my_package",
    package_name = "my_ros_package",
    executables = {
        ":talker_binary": "talker",
        ":listener_binary": "listener",
    },
    package_xml = "package.xml",  # Optional
)
```

Then in a launch file:

```python
from launch_ros.actions import Node

Node(package='my_ros_package', executable='talker')
```

The ros_package target must be included in the launch target's data attribute
for the package to be discoverable at runtime.
""",
)
