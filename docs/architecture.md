# Architecture

One process, `teleop.py`, is both a ROS node and an HTTP server on port 8081.
There is no build step and no framework: the page is a single static file the
server hands out, and every exchange after that is JSON or a JPEG.

## Contents

- [Components](#components)
- [Data pipeline](#data-pipeline)
- [Module layout](#module-layout)
- [Threading](#threading)
- [Design notes](#design-notes)

## Components

| Piece | Responsibility |
|---|---|
| `TeleopNode` | The ROS side: publishes `/drive` on a timer, subscribes to the camera, depth, lidar, odometry and detection topics, and keeps the shared state current. |
| `Handler` | The HTTP side: serves the page, the two preview JPEGs, the state JSON, and the log endpoints. |
| `teleop.html` | The whole browser side. Holds the button and key state, posts commands, and draws the lidar, the charts and the detection overlay on canvases. |
| `teleop.yaml` | Every tunable. Rewritten in place by Save so its comments survive. |
| Module globals | `_state`, `_params`, `_cmd`, `_hist`, `_rows`, `_marks`, guarded by one `_lock`. |

## Data pipeline

```
/camera/color ─┐
               ├─▶ _image_cb ──▶ resize ──▶ JPEG ──▶ _preview ──────▶ GET /frame
/camera/depth ─┴─▶ _depth_cb ──▶ resize ──▶ _colorize_depth ──▶ JPEG ─▶ GET /depth

/edgetpu/inference ─▶ _det_cb ──▶ normalize against _state['res'] ─┐
/scan ─────────────▶ _scan_cb ─▶ _student_deg, thin by 3 ─────────┤
/odom ─────────────▶ _odom_cb ────────────────────────────────────┼─▶ _state ─▶ GET /state
                                                                   │
browser buttons/keys ─▶ POST /cmd ─▶ _cmd ─┐                       │
                                            ├─▶ _actuate (15 Hz) ──┘
                            teleop.yaml ────┘         │
                                                      ▼
                                              /drive  AckermannDriveStamped
```

`/drive` comes from the timer, never from the browser's post. That is what
lets a silent browser read as released: `_actuate` treats a command older than
`cmd_timeout` as zero, so a closed tab, a dropped link or a page that lost
focus with a key down all coast to a stop rather than driving the last command
forever.

## Module layout

```
teleop.py                    node, HTTP handler, and the pure helpers
teleop.html                  the entire browser side, one file
teleop.yaml                  tunables; Save rewrites the numbers in place
setup.sh                     renders and installs the systemd unit
racecar-webteleop.service.in unit template, @DIR@ substituted at install
tests/test_teleop.py         no ROS graph, camera or browser needed
docs/                        this file and the changelog
VERSION                      tracks the driver release, not its own count
```

The pure helpers are module functions rather than methods precisely so the
tests can reach them without a node: `_colorize_depth`, `_thin`,
`clamp_speed`, `_clean`, `_axis`. `TeleopNode._student_deg` and
`TeleopNode._tick_fps` are static for the same reason.

## Threading

Three threads touch the shared state:

- the ROS executor, in `_spin`, running every subscription callback and the
  `_actuate` timer;
- the HTTP server's worker threads, one per request;
- the main thread, blocked in `serve_forever`.

Everything they share sits behind `_lock`, and nothing holds it across an
encode or a network write. `rclpy.init()` installs signal handlers that shut
the ROS context down but leave `serve_forever` blocked, so the process would
outlive its own SIGTERM until systemd's `TimeoutStopSec` killed it; `main`
takes the signals back and calls `shutdown` off the serving thread.

## Design notes

**Lidar angles.** `LIDAR_MOUNT_YAW_DEG` is the raw scan angle that points at
the car's nose. The Neo mounts its RPLIDAR facing aft, so that is 180, not 0.
Indices and angles come from the message's own `angle_min` and
`angle_increment`, so nothing assumes a sweep width. `wallfollow` carries the
same constant and steers on it.

**Two preview loops, not one.** Each view pulls its own frames, so a stalled
depth stream costs that view its frame rate and leaves the colour view alone.
Both are rate limited and resized before encoding; depth resizes nearest
neighbour, because averaging a valid reading against a zero invents a surface
halfway to a hole in the depth image.

**Detections leave normalized.** `vision_msgs` carries a centre and a size in
source-image pixels. The browser draws the preview at whatever width fits its
column, so the boxes ship as fractions of the frame and are scaled there. They
expire after `DET_TIMEOUT_S`, because a frozen box on a live frame is a lie
about what is out there.

**Colour by contrast.** Orange on the dark surfaces, ember on the light ones,
crimson reserved for stop and fault. The plot ground is tarmac in every view,
so anything drawn there is orange; crimson measures 2.09:1 on it.
