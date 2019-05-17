# actionlib_enhanced

## Overview
This is a custom actionlib client-server implementation.
It can be used in a similar way one can do with the SimpleActionLib. However, it can handle multiple client for the same server, as many goals at any frequency from the same client (multi-threaded requests) without missing tracks of any goal, and gather them in the right order. This means the preemptive state is useless here, as you can choose to handle the goals one after the other, or all at the same time.

## Installation
In a terminal:
```shell
sudo apt install ros-kinetic-actionlib-enhanced
```

## Examples
Examples of how this package can be used are in the `examples` directory.
The server can handle multiple clients without any issue.

