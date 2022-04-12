import inputs
from threading import Thread


class XBoxControllerReader(Thread):
    """
    This class is a thread that allows us to read the inputs of a basic XBox360 controller.
    Note that only one controller can be used.
    """
    _JOY_MAX_VAL = 32768
    _TRIGGER_MAX_VAL = 256

    _key_map = {
        'JOY_LX': 'ABS_X',
        'JOY_LY': 'ABS_Y',
        'JOY_RX': 'ABS_RX',
        'JOY_RY': 'ABS_RY',
        'DPAD_X': 'ABS_HAT0X',
        'DPAD_Y': 'ABS_HAT0Y',
        'X': 'BTN_NORTH',
        'Y': 'BTN_WEST',
        'A': 'BTN_SOUTH',
        'B': 'BTN_EAST',
        'BACK': 'BTN_SELECT',
        'START': 'BTN_START',
        'XBOX': 'BTN_MODE',
        'L_BUMPER': 'BTN_TL',
        'R_BUMPER': 'BTN_TR',
        'L_TRIGGER': 'ABS_Z',
        'R_TRIGGER': 'ABS_RZ',
        'R_THUMB': 'BTN_THUMBR',
        'L_THUMB': 'BTN_THUMBL'
    }

    def __init__(self, callback, accuracy=4, read_rate=0.1, left_joy_tolerance=0.2, right_joy_tolerance=0.2,
                 left_trigger_tolerance=0.05, right_trigger_tolerance=0.05) -> None:
        super().__init__()
        self.callback_function = callback
        self._GAMEPAD = inputs.devices.gamepads[0]

        # Precision (decimal points)
        self._ACCURACY = accuracy
        self._READ_RATE = read_rate

        # Tolerance for detection
        self.left_joy_tolerance = left_joy_tolerance
        self.right_joy_tolerance = right_joy_tolerance

        self.left_trigger_tolerance = left_trigger_tolerance
        self.right_trigger_tolerance = right_trigger_tolerance

        # Joysticks
        # Range from -1.0 to 1.0
        self.joy_lx = 0
        self.joy_ly = 0
        self.joy_rx = 0
        self.joy_ry = 0

        # DPad
        # Value -1, 0, 1
        self.dpad_x = 0
        self.dpad_y = 0

        # Triggers
        # Range 0 to 1
        self.l_trigger = 0
        self.r_trigger = 0

        # Buttons
        # Value 0, 1
        self.x = 0
        self.y = 0
        self.a = 0
        self.b = 0
        self.back = 0
        self.start_ = 0
        self.xbox = 0
        self.l_bumper = 0
        self.r_bumper = 0
        self.l_thumb = 0
        self.r_thumb = 0

        # Running Flag
        self.is_running = False

    def __del__(self):
        self.stop()

    def start(self):
        self.is_running = True
        super().start()

    def run(self):
        while self.is_running:
            try:
                events = self._GAMEPAD.read()
                for index, event in enumerate(events):
                    if event.code is self._key_map["JOY_LX"]:
                        rel_value = round(event.state / self._JOY_MAX_VAL, self._ACCURACY)
                        self.joy_lx = rel_value if abs(rel_value) >= self.left_joy_tolerance else 0
                    elif event.code is self._key_map["JOY_LY"]:
                        rel_value = round(event.state / self._JOY_MAX_VAL, self._ACCURACY)
                        self.joy_ly = rel_value if abs(rel_value) >= self.left_joy_tolerance else 0
                    elif event.code is self._key_map["JOY_RX"]:
                        rel_value = round(event.state / self._JOY_MAX_VAL, self._ACCURACY)
                        self.joy_rx = rel_value if abs(rel_value) >= self.right_joy_tolerance else 0
                    elif event.code is self._key_map["JOY_RY"]:
                        rel_value = round(event.state / self._JOY_MAX_VAL, self._ACCURACY)
                        self.joy_ry = rel_value if abs(rel_value) >= self.right_joy_tolerance else 0
                    elif event.code is self._key_map["DPAD_X"]:
                        self.dpad_x = event.state
                    elif event.code is self._key_map["DPAD_Y"]:
                        self.dpad_y = event.state
                    elif event.code is self._key_map["L_TRIGGER"]:
                        rel_value = round(event.state / self._TRIGGER_MAX_VAL, self._ACCURACY)
                        self.l_trigger = rel_value if rel_value >= self.left_trigger_tolerance else 0
                    elif event.code is self._key_map["R_TRIGGER"]:
                        rel_value = round(event.state / self._TRIGGER_MAX_VAL, self._ACCURACY)
                        self.r_trigger = rel_value if rel_value >= self.right_trigger_tolerance else 0
                    elif event.code is self._key_map["X"]:
                        self.x = event.state
                    elif event.code is self._key_map["Y"]:
                        self.y = event.state
                    elif event.code is self._key_map["A"]:
                        self.a = event.state
                    elif event.code is self._key_map["B"]:
                        self.b = event.state
                    elif event.code is self._key_map["BACK"]:
                        self.back = event.state
                    elif event.code is self._key_map["START"]:
                        self.start_ = event.state
                    elif event.code is self._key_map["XBOX"]:
                        self.xbox = event.state
                    elif event.code is self._key_map["L_BUMPER"]:
                        self.l_bumper = event.state
                    elif event.code is self._key_map["R_BUMPER"]:
                        self.r_bumper = event.state
                    elif event.code is self._key_map["L_THUMB"]:
                        self.l_thumb = event.state
                    elif event.code is self._key_map["R_THUMB"]:
                        self.r_thumb = event.state

                self.callback_function(self.read())
            except Exception:
                ...


    def stop(self):
        self.is_running = False
        if self.is_alive():
            self.join()

    def restart(self):
        self.stop()
        self.start()

    def read(self):
        return {
                'JOY_LX': self.joy_lx,
                'JOY_LY': self.joy_ly,
                'JOY_RX': self.joy_rx,
                'JOY_RY': self.joy_ry,
                'L_TRIGGER': self.l_trigger,
                'R_TRIGGER': self.r_trigger,
                'DPAD_X': self.dpad_x,
                'DPAD_Y': self.dpad_y,
                'X': self.x,
                'Y': self.y,
                'A': self.a,
                'B': self.b,
                'L_BUMPER': self.l_bumper,
                'R_BUMPER': self.r_bumper,
                'L_THUMB': self.l_thumb,
                'R_THUMB': self.r_thumb,
                'BACK': self.back,
                'START': self.start_,
                'XBOX': self.xbox,
            }