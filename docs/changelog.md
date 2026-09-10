# Changelog

All notable changes to this dashboard are documented here. The format is based
on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the version
tracks `racecar_neo_ros2_driver` rather than moving on its own: a dashboard
carrying `0.8.1` is the one that release of the driver was tested against.
`setup_dashboards.sh` reads `VERSION` and reports a checkout that does not
match the driver it is being installed by.

## [0.8.1] - 2026-09-10

### Added

- **The depth stream under the colour frame.** The Camera card carries both
  D435i streams at the same width and the same framing, so a feature lines up
  vertically between them. Depth is colorized on the inferno ramp with near
  bright and far dark: one ordered scale, so a step in colour is a step in
  distance and the view survives being read by a colour blind student. The
  ramp starts above black, which leaves pure black to mean "no return" on its
  own; glass, a mirror, and anything past `depth_max_m` are otherwise
  indistinguishable from a wall at the far end. New `depth_topic` and
  `depth_max_m`. An empty `depth_topic` is a car whose camera is colour only
  and drops the pane rather than subscribing to a topic nobody publishes,
  while a configured topic delivering nothing keeps the pane so a dead stream
  stays visible as a fault.
- **A Detections toggle over the colour frame.** Reads
  `vision_msgs/Detection2DArray` from `detections_topic`, default
  `/edgetpu/inference`, and draws each box with its label and confidence.
  Boxes are published in source-image pixels and leave the service normalized,
  so the browser scales them to whatever size it is drawing the preview at and
  they stay correct when the column resizes. Detections older than 1.5 s are
  cleared: a frozen box on a live frame is a lie about what is out there. A
  car with no detector, or no `vision_msgs`, gets the toggle disabled rather
  than a page that fails to load.

### Changed

- **Orange leads the palette, and the palette splits by role.** The two ends
  of the mark's gradient are near inverses in contrast: crimson measures 7.8:1
  on white and 2.1:1 on tarmac, orange 2.5:1 on white and 6.5:1 on tarmac.
  Orange carries the brand and the interactive role on the dark surfaces,
  ember `#A55312` (the same 26 degree hue held down to L 0.36, 5.5:1 on white)
  on the light ones. Crimson is kept for STOP and TIMED OUT: on a dashboard
  that drives a real car one colour should mean stop and nothing else. The
  state box read red while driving normally and shared that red with the STOP
  button; it is ember while moving and crimson only on a fault.
- **Row 1 is the camera stack, the lidar, then the encoder and the logs.** The
  camera column narrows to 400 px so the stacked pair stands about as tall as
  the lidar beside it, and the Logs card moves up into the width that opens on
  the right.
- Documentation retargeted from ROS Humble to Jazzy, and from the LakiBeam to
  this car's RPLIDAR.

### Fixed

- **The lidar view was 180 degrees out.** The scan mapping assumed a
  forward-facing lidar (`student = -raw`), which is the neoracer's 270 degree
  mount. The Neo mounts its RPLIDAR facing aft over 360 degrees at 1080
  points, so the nose sits at the raw 180 edge. `LIDAR_MOUNT_YAW_DEG` carries
  the offset and the angle comes from the message's own `angle_min` and
  `angle_increment`, so a 360 and a 270 degree scan both work and a
  forward-facing lidar needs only `0.0`. Same constant and convention as
  `wallfollow`, which steers on it. This resolves the v0.8.0 known limitation
  for this dashboard; it never affected driving, because teleop's input is
  manual.
- **Half of what the charts drew was invisible.** The plot ground is tarmac
  `#231F20`, and crimson on it measures 2.09:1, under the 3:1 floor for a
  graphical element. The commanded-throttle trace and the car marker on the
  lidar view were both drawn in it. Both are orange now, at 6.5:1. The
  slider-change markers were the reverse problem, gold at 9.4:1 with an 8 px
  glow, drawn louder than the data they annotate; they are a plain neutral
  dashed rule. The third log series moves from gold to cyan `#6FC9D4`, since
  gold and orange are adjacent hues and the two traces were no longer
  separable.
- **Chart labels were scaled up with the canvas.** The chart is 420x160
  stretched to the card width, which scales its text along with everything
  else: the axis labels came out half again too big and ran into the trace and
  into each other. The backing store is matched to the drawn box before each
  pass.

## [0.8.0] - 2026-09-09

### Changed

- Retargeted for the RACECAR Neo: port 8081, the `racecar-webteleop` unit
  name, ROS Jazzy paths, and the RACECAR Neo mark, wordmark and favicon in
  place of the Neobotics set. The unit is `racecar-webteleop` rather than
  `racecar-teleop`, which is the driver's core ROS stack and would collide.

## [0.4.0] - 2026-09-04

- Forked from the Neobotics Foundation web teleop dashboard.
