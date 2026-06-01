from .drive import LEFT, RIGHT


def spin_in_place(motion, direction, seconds, duty_cycle=20):
    """Self-rotation: rotate the robot around its own centre."""
    motion.turn_in_place(direction, seconds, duty_cycle=duty_cycle)


def circle_cube(
    motion,
    side=RIGHT,
    turn_seconds=0.45,
    short_forward_seconds=0.8,
    long_forward_seconds=1.1,
    safety_check=None,
):
    """Orbit a cube by composing self-rotations and safe forward arcs."""
    if side == RIGHT:
        spin_in_place(motion, RIGHT, turn_seconds)
        motion.safe_move_forward_for(short_forward_seconds, safety_check)
        spin_in_place(motion, LEFT, turn_seconds)
        motion.safe_move_forward_for(long_forward_seconds, safety_check)
        spin_in_place(motion, LEFT, turn_seconds)
        motion.safe_move_forward_for(short_forward_seconds, safety_check)
        spin_in_place(motion, RIGHT, turn_seconds)
    else:
        spin_in_place(motion, LEFT, turn_seconds)
        motion.safe_move_forward_for(short_forward_seconds, safety_check)
        spin_in_place(motion, RIGHT, turn_seconds)
        motion.safe_move_forward_for(long_forward_seconds, safety_check)
        spin_in_place(motion, RIGHT, turn_seconds)
        motion.safe_move_forward_for(short_forward_seconds, safety_check)
        spin_in_place(motion, LEFT, turn_seconds)

