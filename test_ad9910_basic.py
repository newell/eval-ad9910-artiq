from artiq.coredevice.ad9910_basic import (
    _AD9910_REG_FTW,
    _AD9910_REG_PROFILE0,
    RAM_DEST_FTW,
    RAM_MODE_RAMPUP,
)
from artiq.experiment import *
from artiq.test.hardware_testbench import ExperimentCase


class AD9910BasicExp(EnvExperiment):
    def build(self, runner):
        self.setattr_device("core")
        self.dev = self.get_device("dds0")
        self.runner = runner

    def run(self):
        getattr(self, self.runner)()

    @kernel
    def instantiate(self):
        pass

    @kernel
    def init(self):
        self.core.break_realtime()
        self.dev.init()

    @kernel
    def set_pow_get_pow(self):
        self.core.break_realtime()
        self.dev.init()
        set_pow = 0xF0F0
        self.dev.set_pow(set_pow)

        get_pow = self.dev.get_pow()
        if get_pow == 0:
            for _ in range(10):
                self.core.break_realtime()
                get_pow = self.dev.get_pow()
                if get_pow != 0:
                    break

        self.set_dataset("get_pow", get_pow)
        self.set_dataset("set_pow", set_pow)

    @kernel
    def set_get(self):
        self.core.break_realtime()
        self.dev.init()
        f = 81.2345 * MHz
        p = 0.33
        a = 0.89
        self.dev.set(frequency=f, phase=p, amplitude=a)

        self.core.break_realtime()
        ftw, pow_, asf = self.dev.get_mu()
        self.core.break_realtime()

        self.set_dataset("ftw_get", ftw)
        self.set_dataset("ftw_set", self.dev.frequency_to_ftw(f))
        self.set_dataset("pow_get", pow_)
        self.set_dataset("pow_set", self.dev.turns_to_pow(p))
        self.set_dataset("asf_get", asf)
        self.set_dataset("asf_set", self.dev.amplitude_to_asf(a))

    @kernel
    def set_get_io_update_regs(self):
        self.core.break_realtime()
        self.dev.init()
        f = 81.2345 * MHz
        p = 0.33
        a = 0.89
        self.dev.set_frequency(f)
        self.dev.set_phase(p)
        self.dev.set_amplitude(a)

        self.core.break_realtime()
        ftw = self.dev.get_ftw()
        self.core.break_realtime()
        pow_ = self.dev.get_pow()
        self.core.break_realtime()
        asf = self.dev.get_asf()

        self.set_dataset("ftw_set", self.dev.frequency_to_ftw(f))
        self.set_dataset("ftw_get", ftw)
        self.set_dataset("pow_set", self.dev.turns_to_pow(p))
        self.set_dataset("pow_get", pow_)
        self.set_dataset("asf_set", self.dev.amplitude_to_asf(a))
        self.set_dataset("asf_get", asf)

    @kernel
    def read_write64(self):
        self.core.break_realtime()
        self.dev.init()
        lo = 0x12345678
        hi = 0x09ABCDEF
        self.dev.write64(_AD9910_REG_PROFILE0, hi, lo)
        self.dev.io_update.pulse_mu(8)
        read = self.dev.read64(_AD9910_REG_PROFILE0)
        self.set_dataset("write", (int64(hi) << 32) | lo)
        self.set_dataset("read", read)

    @kernel
    def set_speed(self):
        self.core.break_realtime()
        self.dev.init()
        f = 81.2345 * MHz
        n = 10
        t0 = self.core.get_rtio_counter_mu()
        for i in range(n):
            self.dev.set(frequency=f, phase=0.33, amplitude=0.89)
        self.set_dataset(
            "dt", self.core.mu_to_seconds(self.core.get_rtio_counter_mu() - t0) / n
        )

    @kernel
    def set_speed_mu(self):
        self.core.break_realtime()
        self.dev.init()
        n = 10
        t0 = self.core.get_rtio_counter_mu()
        for i in range(n):
            self.dev.set_mu(0x12345678, 0x1234, 0x4321)
        self.set_dataset(
            "dt", self.core.mu_to_seconds(self.core.get_rtio_counter_mu() - t0) / n
        )

    @kernel
    def profile_readback(self):
        self.core.break_realtime()
        self.dev.init()
        for i in range(8):
            self.dev.set_mu(ftw=i, profile=i)
        ftw = [0] * 8
        for i in range(8):
            self.dev.set_profile(i)
            # If PROFILE is not alligned to SYNC_CLK a multi-bit change
            # doesn't transfer cleanly. Use IO_UPDATE to load the profile
            # again.
            self.dev.io_update.pulse_mu(8)
            ftw[i] = self.dev.read32(_AD9910_REG_FTW)
            delay(100 * us)
        self.set_dataset("ftw", ftw)

    @kernel
    def ram_write(self):
        n = 1 << 10
        write = [0] * n
        for i in range(n):
            write[i] = i | (i << 16)
        read = [0] * n

        self.core.break_realtime()
        self.dev.init()
        self.dev.set_cfr1(ram_enable=0)
        self.dev.io_update.pulse_mu(8)
        self.dev.set_profile_ram(
            start=0, end=0 + n - 1, step=1, profile=0, mode=RAM_MODE_RAMPUP
        )
        self.dev.set_profile(0)
        self.dev.io_update.pulse_mu(8)
        delay(1 * ms)
        self.dev.write_ram(write)
        delay(1 * ms)
        self.dev.read_ram(read)
        self.set_dataset("w", write)
        self.set_dataset("r", read)

    @kernel
    def ram_read_overlapping(self):
        write = [0] * 989
        for i in range(len(write)):
            write[i] = i
        read = [0] * 100
        offset = 367

        self.core.break_realtime()
        self.dev.init()
        self.dev.set_cfr1(ram_enable=0)
        self.dev.io_update.pulse_mu(8)

        self.dev.set_profile_ram(
            start=0, end=0 + len(write) - 1, step=1, profile=0, mode=RAM_MODE_RAMPUP
        )
        self.dev.set_profile_ram(
            start=offset,
            end=offset + len(read) - 1,
            step=1,
            profile=1,
            mode=RAM_MODE_RAMPUP,
        )

        self.dev.set_profile(0)
        self.dev.io_update.pulse_mu(8)
        delay(1 * ms)
        self.dev.write_ram(write)
        delay(1 * ms)
        self.dev.set_profile(1)
        self.dev.io_update.pulse_mu(8)
        self.dev.read_ram(read)

        # RAM profile addresses are apparently aligned
        # to the last address of the RAM
        start = len(write) - offset - len(read)
        end = len(write) - offset
        self.set_dataset("w", write[start:end])
        self.set_dataset("r", read)

    @kernel
    def ram_exec(self):
        ftw0 = [0x12345678] * 2
        ftw1 = [0x55AAAA55] * 2
        self.core.break_realtime()
        self.dev.init()
        self.dev.set_cfr1(ram_enable=0)
        self.dev.io_update.pulse_mu(8)

        self.dev.set_profile_ram(
            start=100, end=100 + len(ftw0) - 1, step=1, profile=3, mode=RAM_MODE_RAMPUP
        )
        self.dev.set_profile_ram(
            start=200, end=200 + len(ftw1) - 1, step=1, profile=4, mode=RAM_MODE_RAMPUP
        )

        self.dev.set_profile(3)
        self.dev.io_update.pulse_mu(8)
        self.dev.write_ram(ftw0)

        self.dev.set_profile(4)
        self.dev.io_update.pulse_mu(8)
        self.dev.write_ram(ftw1)

        self.dev.set_cfr1(ram_enable=1, ram_destination=RAM_DEST_FTW)
        self.dev.io_update.pulse_mu(8)

        self.dev.set_profile(3)
        self.dev.io_update.pulse_mu(8)
        ftw0r = self.dev.read32(_AD9910_REG_FTW)
        delay(100 * us)

        self.dev.set_profile(4)
        self.dev.io_update.pulse_mu(8)
        ftw1r = self.dev.read32(_AD9910_REG_FTW)

        self.set_dataset("ftw", [ftw0[0], ftw0r, ftw1[0], ftw1r])

    @kernel
    def ram_convert_frequency(self):
        freq = [33 * MHz] * 2
        ram = [0] * len(freq)
        self.dev.frequency_to_ram(freq, ram)

        self.core.break_realtime()
        self.dev.init()
        self.dev.set_cfr1(ram_enable=0)
        self.dev.io_update.pulse_mu(8)
        self.dev.set_profile_ram(
            start=100, end=100 + len(ram) - 1, step=1, profile=6, mode=RAM_MODE_RAMPUP
        )
        self.dev.set_profile(6)
        self.dev.io_update.pulse_mu(8)
        self.dev.write_ram(ram)
        self.dev.set_cfr1(ram_enable=1, ram_destination=RAM_DEST_FTW)
        self.dev.io_update.pulse_mu(8)
        ftw_read = self.dev.read32(_AD9910_REG_FTW)
        self.set_dataset("ram", ram)
        self.set_dataset("ftw_read", ftw_read)
        self.set_dataset("freq", freq)

    @kernel
    def ram_convert_powasf(self):
        amplitude = [0.1, 0.9]
        turns = [0.3, 0.5]
        ram = [0] * 2
        self.dev.turns_amplitude_to_ram(turns, amplitude, ram)
        self.set_dataset("amplitude", amplitude)
        self.set_dataset("turns", turns)
        self.set_dataset("ram", ram)


class AD9910Test(ExperimentCase):
    def test_instantiate(self):
        self.execute(AD9910BasicExp, "instantiate")

    def test_init(self):
        self.execute(AD9910BasicExp, "init")

    def test_set_pow_get_pow(self):
        self.execute(AD9910BasicExp, "set_pow_get_pow")
        get_pow = self.dataset_mgr.get("get_pow")
        set_pow = self.dataset_mgr.get("set_pow")
        self.assertEqual(get_pow, set_pow)

    # ERRORING
    # def test_set_get(self):
    #     self.execute(AD9910BasicExp, "set_get")
    #     for attr in ["ftw", "pow", "asf"]:
    #         with self.subTest(attribute=attr):
    #             get = self.dataset_mgr.get("{}_get".format(attr))
    #             set_ = self.dataset_mgr.get("{}_set".format(attr))
    #             self.assertEqual(get, set_)

    # def test_set_get_io_update_regs(self):
    #     self.execute(AD9910BasicExp, "set_get_io_update_regs")
    #     for attr in ["ftw", "pow", "asf"]:
    #         with self.subTest(attribute=attr):
    #             get = self.dataset_mgr.get("{}_get".format(attr))
    #             set_ = self.dataset_mgr.get("{}_set".format(attr))
    #             self.assertEqual(get, set_)

    def test_read_write64(self):
        self.execute(AD9910BasicExp, "read_write64")
        write = self.dataset_mgr.get("write")
        read = self.dataset_mgr.get("read")
        self.assertEqual(hex(write), hex(read))

    def test_set_speed(self):
        self.execute(AD9910BasicExp, "set_speed")
        dt = self.dataset_mgr.get("dt")
        print(dt)
        self.assertLess(dt, 70 * us)

    def test_set_speed_mu(self):
        self.execute(AD9910BasicExp, "set_speed_mu")
        dt = self.dataset_mgr.get("dt")
        print(dt)
        self.assertLess(dt, 11 * us)

    # ERRORING
    # def test_profile_readback(self):
    #     self.execute(AD9910BasicExp, "profile_readback")
    #     self.assertEqual(self.dataset_mgr.get("ftw"), list(range(8)))

    def test_ram_write(self):
        self.execute(AD9910BasicExp, "ram_write")
        read = self.dataset_mgr.get("r")
        write = self.dataset_mgr.get("w")
        self.assertEqual(len(read), len(write))
        self.assertEqual(read, write)

    def test_ram_read_overlapping(self):
        self.execute(AD9910BasicExp, "ram_read_overlapping")
        read = self.dataset_mgr.get("r")
        write = self.dataset_mgr.get("w")
        self.assertEqual(len(read), 100)
        self.assertEqual(read, write)

    # ERRORING
    # def test_ram_exec(self):
    #     self.execute(AD9910BasicExp, "ram_exec")
    #     ftw = self.dataset_mgr.get("ftw")
    #     self.assertEqual(ftw[0], ftw[1])
    #     self.assertEqual(ftw[2], ftw[3])

    def test_ram_convert_frequency(self):
        exp = self.execute(AD9910BasicExp, "ram_convert_frequency")
        ram = self.dataset_mgr.get("ram")
        ftw_read = self.dataset_mgr.get("ftw_read")
        self.assertEqual(ftw_read, ram[0])
        freq = self.dataset_mgr.get("freq")
        self.assertEqual(ftw_read, exp.dev.frequency_to_ftw(freq[0]))
        self.assertAlmostEqual(freq[0], exp.dev.ftw_to_frequency(ftw_read), delta=0.25)

    # ERRORING
    # def test_ram_convert_powasf(self):
    #     exp = self.execute(AD9910BasicExp, "ram_convert_powasf")
    #     ram = self.dataset_mgr.get("ram")
    #     amplitude = self.dataset_mgr.get("amplitude")
    #     turns = self.dataset_mgr.get("turns")
    #     for i in range(len(ram)):
    #         self.assertEqual((ram[i] >> 16) & 0xFFFF, exp.dev.turns_to_pow(turns[i]))
    #         self.assertEqual(ram[i] & 0xFFFF, exp.dev.amplitude_to_asf(amplitude[i]))
