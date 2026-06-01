from .drive import LEFT, RIGHT
from .spin import spin_in_place


def circle_cube(
    motion,
    side=RIGHT,
    turn_seconds=0.45,
    short_forward_seconds=0.8,
    long_forward_seconds=1.1,
    safety_check=None,
):
    """Break a cube orbit into self-rotations and short forward arcs."""
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
