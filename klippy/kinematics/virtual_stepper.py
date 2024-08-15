from stepper import PrinterRail


class VirtualPrinterRail(PrinterRail):

    def __init__(
            self,
            short_name: str,
            units_in_radians=False
    ):
        self._short_name = short_name
        self._commanded_position = 10.0

        # Primary stepper and endstop
        self.stepper_units_in_radians = units_in_radians
        self.steppers = []
        self.endstops = []
        self.endstop_map = {}
        # self.add_extra_stepper(config)
        # mcu_stepper = self.steppers[0]

        # implemented
        # self.get_name = mcu_stepper.get_name
        # self.get_commanded_position = mcu_stepper.get_commanded_position
        # self.calc_position_from_coord = mcu_stepper.calc_position_from_coord

        # Primary endstop position
        self.position_endstop = 1.0

        # Axis range
        self.position_min = 0.0
        self.position_max = 100.0

        # Homing mechanics
        self.homing_speed = 5.0
        self.second_homing_speed = self.homing_speed / 2.
        self.homing_retract_speed = self.homing_speed
        self.homing_retract_dist = 5.0
        self.homing_positive_dir = False

    def get_name(self) -> str:
        return self._short_name

    def get_commanded_position(self) -> float:
        return self._commanded_position

    def calc_position_from_coord(self, coord):

        if self._short_name == 'x':
            position = coord[0]
        elif self._short_name == 'y':
            position = coord[1]
        elif self._short_name == 'z':
            position = coord[2]
        else:
            raise RuntimeError(f'Unknown virtual stepper axis: {self._short_name}')

        return position

    def get_steppers(self):
        return []

    def get_endstops(self):
        return []

    def add_extra_stepper(self, config):
        pass

    def setup_itersolve(self, alloc_func, *params):
        pass

    def generate_steps(self, flush_time):
        pass

    def set_trapq(self, trapq):
        pass

    def set_position(self, coord):
        pass
