from machine import Pin, PWM, reset
from utime import ticks_us

class DistanceSensor:
    def __init__(self, echo_id, trig_id, trig_freq=12):
        assert 10 < trig_freq < 16
        self.trig_pin = PWM(Pin(trig_id), freq=trig_freq, duty_ns=10_000)
        self.echo_pin = Pin(echo_id, Pin.IN, Pin.PULL_DOWN)
        self.echo_pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.decode_distance)
        self.distance = None  # unit: m
        self.echo_tic = ticks_us()  # echo pulled up stamp

    def decode_distance(self, pin):
        if pin.value():
            self.echo_tic = ticks_us()
        else:
            dt = ticks_us() - self.echo_tic
            if dt < 100:  # too close
                self.distance = 0.0
            elif 100 <= dt < 38000:
                self.distance = dt / 58 / 100  # m
            else:  # nothing detected, or not functional
                self.distance = None

if __name__ == "__main__":
    from utime import sleep_ms

    sensor = DistanceSensor(echo_id=8, trig_id=9, trig_freq=15)

    try:
        while True:
            print(f"Distance: {sensor.distance} m")
            sleep_ms(100)
    except KeyboardInterrupt:
        reset()
