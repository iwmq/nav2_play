This is a collection of custom plugins for playing with ROS2 navigation2.


Most of the plugins will start with navigation's already-existing ones,
and with progressive modification to benifit my learning.

How to run unit testing?
---
First, `cd ~/<workspace>/build/nav2_play_plugins`, then `ctest -VV -R test_`.

Using `-R test_` tell `ctest` only run tests starting with `test_`, so copyright
check and style check can be skipped.

To be honest, the `uncrustify` style linter is very annoying.