# Webots world setup

`turtlebot3_mapping.wbt` is an R2025a-compatible adaptation of Cyberbotics' Apache-2.0 licensed
`turtlebot3_burger.wbt` sample. Its arena and obstacles use only Webots base nodes, avoiding optional
`webots://projects` resources that may be absent from some macOS packages. The only remote dependency
is Cyberbotics' official tagged R2025a `TurtleBot3Burger` PROTO.

The macOS launchers switch the same TurtleBot3 node between `mapping_controller` and
`localization_controller` automatically.
