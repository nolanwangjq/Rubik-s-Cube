# Rubik's Cube Robot

This project controls a Raspberry Pi robot for the Rubik's-cube obstacle course.
The runtime is split into three layers:

```text
main.py
  state machine and course-level decisions

motion/
  low-level motor control and cube-orbit motion primitives

perception/
  camera color detection, I2C distance sensing, and safety checks
```

## Course State Machine

`main.py` owns the high-level course flow:

```text
FIRST_STRAIGHT
  -> drive from the first cube side while tracking the green cube

ORBIT_GREEN
  -> circle the green cube using self-rotations and forward arcs

THIRD_STRAIGHT
  -> pass the third cube side and drive to the finish

DONE
```

The state machine calls `perception` first for safety and then calls `motion`
to move the robot.

## Motion Layer

`motion/drive.py`

- Initializes GPIO and PWM.
- Provides forward motion, stop, left/right steering, in-place turn, and safe timed forward motion.

`motion/course.py`

- `spin_in_place(...)`: robot self-rotation.
- `circle_cube(...)`: cube orbit broken into self-rotations plus safe forward arcs.

`motion/rotate/`

- Encoder-based rotation experiments and manual rotate tests.
- Kept separate from the main course flow.

## Perception Layer

`perception/distance.py`

- Starts a background thread.
- Talks to the I2C distance module at address `0x74`.
- Sends command `0xB0` and reads distance from registers `0x2` and `0x3`.

`perception/vision.py`

- Starts a camera thread.
- Detects red, green, and yellow cubes by HSV masks.
- Publishes target offset `diff` for PID steering.

`perception/safety.py`

- Combines camera detections and distance readings.
- Detects red/yellow cube danger, green target arrival, and wall or unknown obstacle danger.

## Main Tuning Parameters

Most field tuning is in `main.py`:

```python
FINAL_STRAIGHT_SECONDS
ORBIT_TURN_SECONDS
ORBIT_SHORT_FORWARD_SECONDS
ORBIT_LONG_FORWARD_SECONDS
TARGET_DISTANCE
WALL_DANGER_DISTANCE
DANGER_CENTER_MARGIN
DANGER_MIN_Y
```

Run on the robot with:

```bash
python3 main.py
```

