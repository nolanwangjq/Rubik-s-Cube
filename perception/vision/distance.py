import threading

import wiringpi as wpi


class DistanceSensor:
    def __init__(self, address=0x74, command=0xB0):
        self.address = address
        self.command = command
        self.distance_cm = 0
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        handle = wpi.wiringPiI2CSetup(self.address)
        while not self._stop.is_set():
            wpi.wiringPiI2CWriteReg8(handle, 0x2, self.command)
            wpi.delay(33)
            high_byte = wpi.wiringPiI2CReadReg8(handle, 0x2)
            low_byte = wpi.wiringPiI2CReadReg8(handle, 0x3)
            dist_mm = (high_byte << 8) + low_byte
            self.distance_cm = dist_mm / 10.0
            print("Distance:", self.distance_cm, "cm")

    def get_distance(self):
        return self.distance_cm

    def stop(self):
        self._stop.set()

