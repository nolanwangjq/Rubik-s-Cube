from motions import RIGHT


class SafetyMonitor:
    def __init__(
        self,
        vision,
        distance_sensor,
        motion,
        target_color="green",
        safety_colors=None,
        cube_colors=None,
        target_distance=20,
        target_center_margin=120,
        wall_danger_distance=12,
        danger_min_area=500,
        danger_center_margin=110,
        danger_min_y=180,
        image_center=320,
        default_turn_side=RIGHT,
    ):
        self.vision = vision
        self.distance_sensor = distance_sensor
        self.motion = motion
        self.target_color = target_color
        self.safety_colors = safety_colors or ["red", "yellow"]
        self.cube_colors = cube_colors or ["red", "green", "yellow"]
        self.target_distance = target_distance
        self.target_center_margin = target_center_margin
        self.wall_danger_distance = wall_danger_distance
        self.danger_min_area = danger_min_area
        self.danger_center_margin = danger_center_margin
        self.danger_min_y = danger_min_y
        self.image_center = image_center
        self.default_turn_side = default_turn_side

    def get_forward_danger(self, colors=None):
        colors = colors or self.safety_colors
        for color in colors:
            det = self.vision.get_detection(color)
            if det is None:
                continue
            center_x, center_y = det["center"]
            if det["area"] < self.danger_min_area:
                continue
            if abs(center_x - self.image_center) <= self.danger_center_margin and center_y >= self.danger_min_y:
                return color, det
        return None, None

    def avoid_immediate_danger(self, colors=None):
        danger_color, danger = self.get_forward_danger(colors)
        if danger is not None:
            center_x = danger["center"][0]
            print("Avoid", danger_color)
            if center_x < self.image_center:
                self.motion.turn_right_not_in_place()
            else:
                self.motion.turn_left_not_in_place()
            return True

        distance = self.distance_sensor.get_distance()
        if 0 < distance <= self.wall_danger_distance:
            print("Avoid wall or unknown obstacle")
            self.motion.stop()
            self.motion.turn_in_place(self.default_turn_side, 0.2)
            return True

        return False

    def target_reached(self):
        target = self.vision.get_detection(self.target_color)
        if target is None:
            return False
        if self.get_forward_danger(self.safety_colors)[1] is not None:
            return False
        center_x = target["center"][0]
        distance = self.distance_sensor.get_distance()
        return distance <= self.target_distance and abs(center_x - self.image_center) <= self.target_center_margin

    def can_forward(self, safe_distance=20):
        return self.distance_sensor.get_distance() > safe_distance

