import time

from motion import RIGHT, MotionController, circle_cube
from perception import DistanceSensor, SafetyMonitor, VisionSystem


FIRST_STRAIGHT = "first_straight"
ORBIT_GREEN = "orbit_green"
THIRD_STRAIGHT = "third_straight"
DONE = "done"

SECOND_CUBE_COLOR = "green"
SECOND_CUBE_SIDE = RIGHT
SAFETY_COLORS = ["red", "yellow"]
CUBE_COLORS = ["red", "green", "yellow"]

IMAGE_CENTER = 320
LEFT_SPEED = 30
RIGHT_SPEED = 30
CONTROL_LIMIT = 30

KP = 1.8
KI = 0.1
KD = 0.001

FINAL_STRAIGHT_SECONDS = 3.0
ORBIT_TURN_SECONDS = 0.45
ORBIT_SHORT_FORWARD_SECONDS = 0.8
ORBIT_LONG_FORWARD_SECONDS = 1.1

TARGET_DISTANCE = 20
TARGET_CENTER_MARGIN = 120
WALL_DANGER_DISTANCE = 12
MIN_BLOB_AREA = 300
DANGER_MIN_AREA = 500
DANGER_CENTER_MARGIN = 110
DANGER_MIN_Y = 180


def clamp(value, limit):
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


def update_pid(diff, errors, adjusts):
    errors[0] = errors[1]
    errors[1] = errors[2]
    errors[2] = diff

    adjusts[0] = adjusts[1]
    adjusts[1] = adjusts[2]
    adjusts[2] = adjusts[1] + KP * (errors[2] - errors[1]) + KI * errors[2] + KD * (
        errors[2] - 2 * errors[1] + errors[0]
    )
    adjusts[2] = clamp(adjusts[2], CONTROL_LIMIT)
    print(adjusts[2])
    return adjusts[2]


def drive_from_first_cube_side(motion, vision, safety, errors, adjusts):
    adjust = update_pid(vision.get_diff(), errors, adjusts)

    if safety.target_reached():
        motion.stop()
        return ORBIT_GREEN

    if safety.avoid_immediate_danger(SAFETY_COLORS):
        return FIRST_STRAIGHT

    if safety.can_forward():
        if adjust > 10:
            motion.turn_right_not_in_place()
        elif adjust < -10:
            motion.turn_left_not_in_place()
        else:
            motion.move_forward()
    else:
        motion.stop()
        motion.turn_in_place(SECOND_CUBE_SIDE, 0.2)

    return FIRST_STRAIGHT


def orbit_green_cube(motion, safety):
    print("Start circling the second cube!")
    time.sleep(1)
    circle_cube(
        motion,
        side=SECOND_CUBE_SIDE,
        turn_seconds=ORBIT_TURN_SECONDS,
        short_forward_seconds=ORBIT_SHORT_FORWARD_SECONDS,
        long_forward_seconds=ORBIT_LONG_FORWARD_SECONDS,
        safety_check=lambda: safety.avoid_immediate_danger(CUBE_COLORS),
    )
    return THIRD_STRAIGHT


def pass_third_cube_side_and_finish(motion, safety):
    print("Final straight!")
    motion.safe_move_forward_for(
        FINAL_STRAIGHT_SECONDS,
        safety_check=lambda: safety.avoid_immediate_danger(CUBE_COLORS),
    )
    return DONE


def main():
    motion = MotionController(left_speed=LEFT_SPEED, right_speed=RIGHT_SPEED)
    distance_sensor = DistanceSensor()
    vision = VisionSystem(
        target_color=SECOND_CUBE_COLOR,
        watch_colors=CUBE_COLORS,
        image_center=IMAGE_CENTER,
        min_blob_area=MIN_BLOB_AREA,
    )
    safety = SafetyMonitor(
        vision=vision,
        distance_sensor=distance_sensor,
        motion=motion,
        target_color=SECOND_CUBE_COLOR,
        safety_colors=SAFETY_COLORS,
        cube_colors=CUBE_COLORS,
        target_distance=TARGET_DISTANCE,
        target_center_margin=TARGET_CENTER_MARGIN,
        wall_danger_distance=WALL_DANGER_DISTANCE,
        danger_min_area=DANGER_MIN_AREA,
        danger_center_margin=DANGER_CENTER_MARGIN,
        danger_min_y=DANGER_MIN_Y,
        image_center=IMAGE_CENTER,
        default_turn_side=SECOND_CUBE_SIDE,
    )

    print("ready")
    input()

    distance_sensor.start()
    vision.start()
    time.sleep(3)

    state = FIRST_STRAIGHT
    errors = [0.0] * 3
    adjusts = [0.0] * 3

    try:
        while state != DONE:
            if state == FIRST_STRAIGHT:
                state = drive_from_first_cube_side(motion, vision, safety, errors, adjusts)
            elif state == ORBIT_GREEN:
                state = orbit_green_cube(motion, safety)
            elif state == THIRD_STRAIGHT:
                state = pass_third_cube_side_and_finish(motion, safety)
    except KeyboardInterrupt:
        print("END!!!")
    finally:
        distance_sensor.stop()
        vision.stop()
        motion.cleanup()


if __name__ == "__main__":
    main()
