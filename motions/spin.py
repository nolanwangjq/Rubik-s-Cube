def spin_in_place(motion, direction, seconds, duty_cycle=20):
    """Self-rotation: turn the chassis around its own centre."""
    motion.turn_in_place(direction, seconds, duty_cycle=duty_cycle)

