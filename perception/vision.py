import threading

import cv2
import numpy as np

from motions import RIGHT


LOWER_RANGES = {
    "red": np.array([0, 80, 50]),
    "green": np.array([35, 43, 46]),
    "blue": np.array([100, 40, 40]),
    "yellow": np.array([20, 80, 80]),
}

UPPER_RANGES = {
    "red": np.array([8, 255, 220]),
    "green": np.array([77, 255, 255]),
    "blue": np.array([140, 255, 255]),
    "yellow": np.array([35, 255, 255]),
}


def get_corner_from_contour(contour, corner):
    bottom_point = tuple(contour[contour[:, :, 1].argmax()][0])
    bottom_points = [pt[0] for pt in contour if pt[0][1] == bottom_point[1]]
    if corner == RIGHT:
        return max(bottom_points, key=lambda x: x[0])
    return min(bottom_points, key=lambda x: x[0])


class VisionSystem:
    def __init__(
        self,
        target_color="green",
        watch_colors=None,
        color_to_direction=None,
        camera_index=0,
        image_center=320,
        min_blob_area=300,
        show_debug=True,
    ):
        self.target_color = target_color
        self.watch_colors = watch_colors or ["red", "green", "yellow"]
        self.color_to_direction = color_to_direction or {"red": 1, "green": RIGHT, "yellow": RIGHT}
        self.image_center = image_center
        self.min_blob_area = min_blob_area
        self.show_debug = show_debug

        self.cap = cv2.VideoCapture(camera_index)
        self.diff = 0
        self.detections = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            ret, frame = self.cap.read()
            if not ret:
                print("Failed to capture image")
                break

            detections = {}
            target_view = frame[:, :-50]
            target_mask = None
            target_res = None

            for color in self.watch_colors:
                det, view, mask, res = self.get_color_detection(frame, color)
                detections[color] = det
                if color == self.target_color:
                    target_view = view
                    target_mask = mask
                    target_res = res

            target = detections.get(self.target_color)
            with self._lock:
                self.detections = detections
                if target is not None:
                    self.diff = target["corner_x"] - self.image_center

            if self.show_debug:
                self._draw_debug(target_view, target, detections)
                cv2.imshow("img", target_view)
                if target_mask is not None:
                    cv2.imshow("mask", target_mask)
                if target_res is not None:
                    cv2.imshow("res", target_res)
                if cv2.waitKey(30) & 0xFF == 32:
                    break

    def get_color_detection(self, image, color):
        view = image[:, :-50]
        hsv = cv2.cvtColor(view, cv2.COLOR_BGR2HSV)
        kernel = np.ones((3, 3), np.uint8)

        mask = cv2.inRange(hsv, LOWER_RANGES[color], UPPER_RANGES[color])
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=5)
        res = cv2.bitwise_and(view, view, mask=mask)

        cnts, heir = cv2.findContours(mask.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)[-2:]
        if not cnts:
            return None, view, mask, res

        contour = max(cnts, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        if area < self.min_blob_area:
            return None, view, mask, res

        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            return None, view, mask, res

        ((x, y), radius) = cv2.minEnclosingCircle(contour)
        center = (int(moments["m10"] / moments["m00"]), int(moments["m01"] / moments["m00"]))
        direction = self.color_to_direction.get(color, RIGHT)
        corner_point = get_corner_from_contour(contour, direction)

        return {
            "center": center,
            "radius": radius,
            "area": area,
            "corner_x": corner_point[0],
            "bbox": cv2.boundingRect(contour),
        }, view, mask, res

    def _draw_debug(self, view, target, detections):
        if target is not None:
            center = target["center"]
            radius = target["radius"]
            corner_x = target["corner_x"]
            if radius > 5:
                cv2.circle(view, center, int(radius), (0, 255, 255), 2)
                cv2.circle(view, center, 5, (0, 0, 255), -1)
                cv2.circle(view, (corner_x, center[1]), 5, (255, 0, 0), -1)
            print(f"X-coordinate of the corner point: {corner_x}")
            print("The difference:", corner_x - self.image_center)

        for color in ("red", "yellow"):
            danger = detections.get(color)
            if danger is not None:
                cv2.circle(view, danger["center"], 5, (255, 255, 255), -1)

    def get_diff(self):
        with self._lock:
            return self.diff

    def get_detection(self, color):
        with self._lock:
            return self.detections.get(color)

    def stop(self):
        self._stop.set()
        self.cap.release()
        cv2.destroyAllWindows()

