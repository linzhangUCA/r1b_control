from regulated_wheel import RegulatedWheel
from machine import Pin


class DiffDriveController:
    def __init__(
        self, left_wheel_ids: list | tuple, right_wheel_ids: list | tuple
    ) -> None:
        # Configs
        self.left_wheel = RegulatedWheel(*left_wheel_ids)
        self.right_wheel = RegulatedWheel(*right_wheel_ids)
        self.stby_pin = Pin(12, Pin.OUT)
        self.disable()
        # Constants
        self.wheel_sep = 0.122

    def enable(self):
        self.stby_pin.on()

    def disable(self):
        self.stby_pin.off()

    def get_vels(self):
        self.meas_lin_vel = 0.5 * (
            self.left_wheel.meas_lin_vel + self.right_wheel.meas_lin_vel
        )
        self.meas_ang_vel = (
            self.right_wheel.meas_lin_vel - self.left_wheel.meas_lin_vel
        ) / self.wheel_sep
        return self.meas_lin_vel, self.meas_ang_vel

    def set_vels(self, target_lin_vel, target_ang_vel):
        left_wheel_ref_vel = target_lin_vel - 0.5 * (target_ang_vel * self.wheel_sep)
        right_wheel_ref_vel = target_lin_vel + 0.5 * (target_ang_vel * self.wheel_sep)
        self.left_wheel.set_wheel_velocity(left_wheel_ref_vel)
        self.right_wheel.set_wheel_velocity(right_wheel_ref_vel)


if __name__ == "__main__":
    from utime import sleep

    # SETUP
    targ_vels = (
        (0.3, 0.0),
        (0.3, 1.6),
        (0.0, 1.6),
        (-0.3, 1.6),
        (-0.3, 0.0),
        (-0.3, -1.6),
        (0.0, -1.6),
        (0.3, -1.6),
    )
    ddc = DiffDriveController(
        left_wheel_ids=((15, 13, 14), (11, 10)),
        right_wheel_ids=((16, 18, 17), (19, 20)),
    )
    ddc.enable()

    # LOOP
    for tv in targ_vels:
        ddc.set_vels(*tv)
        for _ in range(150):
            mlv, mav = ddc.get_vels()
            print(f"Differential drive lin_vel={mlv}, ang_vel={mav}")
            sleep(0.01)

    # Terminate
    ddc.disable()
    print("motor driver disabled.")
