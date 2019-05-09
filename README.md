# actionlib_enhanced

## Overview
This is a custom actionlib client-server implementation.
It can be used as one can do with the SimpleActionLib. However, it can handle multiple client for the same server, as many goal at any frequency from the same client (multi-threaded requests) without missing tracks of any goal. This means the preemptive state is useless here, as you can choose to handle the goal one after the other, or all at the same time.

## Examples
Examples of how this package can be used are in the directory examples.
The server can handle multiple clients without any issue.

## Installation
`pip install actionlib_enhanced` should work for most users.
