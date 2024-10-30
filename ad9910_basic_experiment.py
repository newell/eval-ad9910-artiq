from artiq.experiment import *


class AD9910BasicTest(EnvExperiment):
    def build(self):
        self.setattr_device("core")
        self.setattr_device("dds0")
        self.setattr_device("io_update")
        self.setattr_device("profile0")
        self.setattr_device("profile1")
        self.setattr_device("profile2")

    @kernel
    def set_pow_get_pow(self):
        self.core.break_realtime()
        self.dds0.init()
        set_pow = 0xAA
        self.dds0.set_pow(set_pow)
        self.io_update.pulse(1 * ms)
        delay(1 * ms)
        self.dds0.get_pow()
        # Odd, it seems I need to read from it
        # twice to get 0xAA showing up in MonInj.
        delay(1 * ms)
        self.dds0.get_pow()

    @kernel
    def run(self):
        self.core.reset()
        self.core.break_realtime()
        self.dds0.init()
        self.dds0.set(frequency=100 * MHz)
        self.io_update.pulse(1 * ms)
        delay(1 * ms)
        self.dds0.set(frequency=120 * MHz, phase=0.5, profile=6)
        self.io_update.pulse(1 * ms)
        delay(1 * ms)
        self.profile0.on()
        self.profile1.on()
        self.profile2.on()
        # Toggle between profile 6 and 7
        toggle = True
        for _ in range(10):
            delay(1 * s)
            if toggle:
                self.profile0.off()
            else:
                self.profile0.on()
            toggle = not toggle
