# Web Teleop Dashboard

Manual driving for the Neoracer from a browser, on port 8087. Four buttons, bound to WASD and the arrow keys, drive the car forward, back, left, and right, two at once for a turn on the move, with the camera, the lidar, and the wheel encoder in view. Two sliders set how hard the buttons push.

The service name is `neoracer-webteleop`, not `neoracer-teleop`: that name belongs to the driver's core stack, which this dashboard rides on.

## Contents

- [Install](#install)
- [Service control](#service-control)
- [Driving](#driving)
- [Dashboard](#dashboard)
- [Parameters](#parameters)
- [Known conflicts on the car](#known-conflicts-on-the-car)
- [Tests](#tests)
- [Safety](#safety)
- [Car specifics](#car-specifics)

## Install

On the car:

```
git clone https://github.com/Neobotics-Foundation-Inc/teleop_dashboard.git
bash teleop_dashboard/setup.sh
```

setup.sh points neoracer-webteleop.service at this checkout wherever it sits and copies nothing, so the repository can live anywhere the racecar user can read. A first install leaves the service stopped and disabled; start it with `bash setup.sh enable`. Dashboard: `http://<car-ip>:8087`.

Re-running setup.sh updates the unit, keeps the car's tuned teleop.yaml, and leaves the enable state alone: a running service restarts on the new code, a stopped one stays stopped.

## Service control

Run on the car, from the checkout:

| Command | Effect |
| --- | --- |
| `bash setup.sh` | install or update the unit; a first install does not start it |
| `bash setup.sh enable` | start now and at every boot |
| `bash setup.sh disable` | stop now and keep off across boots |
| `bash setup.sh restart` | restart; takes port 8087 back first |
| `bash setup.sh remove` | stop, disable, and uninstall the unit; keeps teleop.yaml |

Enable, restart, and an update of a running service clear port 8087 first. A dashboard left over from an earlier install under a different unit name or directory, or any other service on 8087, is stopped through systemd; a `teleop.py` started by hand is signalled directly. Without this the new instance would fail to bind and loop on `Restart=on-failure`.

## Driving

The browser keeps a set of what is held: each pad button under a pointer or finger, each of the eight keys that is down. That set collapses to two axes, forward/back and left/right, each -1, 0, or +1, and the pair is posted to the service as one command on every change and every 100 ms while anything is held. Forward and left pressed together are therefore one command, applied at the same instant; forward and back together cancel to 0.

```
buttons / keys --> held set --> {fwd, turn} --> POST /cmd (on change, 10 Hz while held)
                                                        |
        throttle = fwd  * speed                         v
        steering = turn * angle          /drive at 15 Hz from a timer
                                                        |
                              no command for cmd_timeout -> drive zero
```

`/drive` comes from a timer, not from the browser's posts, so the mux never sees a gap. A command older than `cmd_timeout` counts as released: a closed tab, a dropped link, or a page that lost focus with a key down all coast to zero instead of driving the last command forever. The browser also releases everything on its own when the window loses focus or the tab is hidden, so the timeout is the backstop rather than the first line.

Keys: W, A, S, D or the arrow keys. Key auto-repeat is ignored; a key drives from its keydown to its keyup. Keys are read anywhere on the page except inside a slider or the log picker.

## Dashboard

Top row, the two sensor views side by side at the same width, then the encoder card across the rest:

- Camera view: the live frame.
- Lidar view: scan points around the car, the nose up, range rings at 1, 2, and 3 m, scroll to zoom.
- Encoder card: a state box, STOPPED, FORWARD, FORWARD LEFT, and so on, red while moving, amber on TIMED OUT, and under it the live chart: measured speed from `/odom` in white on its own scale, the commanded throttle in red on a fixed -1 to 1, yellow markers at every slider change. The gap between the two traces is the car's lag and the difference between a commanded fraction and real meters per second.

Second row: the drive pad on the left, the two sliders on the right. The page shows only names and numbers; every explanation is a tooltip. Hover a view, the chart, the state box, a pad button, or a slider to read what it does.

Save and Load write and read teleop.yaml on the car. Reset (top bar) re-reads the yaml. STOP (top bar) releases every button and sets speed to 0.

Save log snapshots what is on the live chart to LogN.csv: the two axes, the throttle and steer sent, and the measured speed. Load log defaults to the latest save and draws measured speed, throttle, and steer together.

## Parameters

teleop.yaml:

| Key | Notes |
| --- | --- |
| speed | 0..1 throttle while forward or back is held; ships at 0.0 |
| angle | 0..1 steering while left or right is held; 1 is full lock |
| cmd_timeout | seconds without a command before the service drives zero; not on the dashboard |
| camera_topic, preview_width, preview_quality | `/camera/color` on the latest driver, `/camera` on older cars; live view size and jpeg quality |

Save rewrites the numbers in place, so the comments in the yaml survive.

## Known conflicts on the car

Anything else publishing /drive will fight this service at the mux and the car will sit still or stutter:

- The wallfollow, pursuit, eps, smartfollow, and linefollow dashboards all publish /drive. Stop them before starting this one: `racecar service stop wallfollow`, and the same for the others.
- neoracer-autonomy runs a twist bridge that idles at zero on /drive. Disable it while using webteleop: `sudo systemctl disable --now neoracer-autonomy`
- A leftover Jupyter kernel that ever created a racecar object keeps publishing /drive. Restart the jupyter service to clear them.

Check with: `ros2 topic info -v /drive` (there should be exactly one publisher: webteleop).

The camera and the lidar are read directly, so the driver's inference node does not need to be running. camlabel reads the same camera topic and does not publish /drive, so it can run alongside.

## Tests

`tests/test_teleop.py` drives the command endpoint and the timer callback with an explicit clock, so no ROS graph, camera, or browser is needed. It covers the axis mapping and its squash to -1, 0, +1, forward and left as one command, the steering negation on /drive, the timeout and its recovery, the speed cap in both directions, the scan thinning, the preview encoding, and the yaml round trip that keeps the comments.

```
source /opt/ros/humble/setup.bash
source /home/racecar/ros2_ws/install/setup.bash
cd teleop_dashboard && pytest -q
```

## Safety

The neoracer mux forwards /drive with no software deadman. The transmitter's SWC/SWB switch is the physical autonomy gate. The shipped yaml has speed at 0.0, so the buttons steer but the car cannot move until the slider is raised. The speed command is hard capped at 1.0 either way in code, and the service drives zero after `cmd_timeout` without a command.

## Car specifics

This package is calibrated for the Neoracer: JPEG frames on /camera/color, LakiBeam lidar angle mapping, steering sign, speed feedback from /odom, ROS Humble paths. The steering sign was verified physically on the sibling dashboards and is the same here.
