"""Test-support code shared by conftest and the component tests.

Kept in a plain module (not conftest) so both pytest-internal and test code
import the SAME FakeSMBus class object; the instance registry must not be
duplicated by double-importing conftest.
"""

import time

I2C_ADDR = 0x2B

# Register values a healthy board would return (ultrasonic 0x0258 = 600 mm,
# line sensor all four channels on).
DEFAULT_REGS = {
    0x1A: [0x58],  # ultrasonic low byte
    0x1B: [0x02],  # ultrasonic high byte
    0x0A: [0x0F],  # line sensor
}


class FakeSMBus:
    """Records every transaction and can inject IOError on reads.

    Mirrors the API the vendor Raspbot_Lib actually calls:
    read_i2c_block_data / write_i2c_block_data / write_byte_data / write_byte.
    Every created instance is registered in ``FakeSMBus.instances`` so tests
    can reach the bus the node opened, without touching /dev/i2c-1.
    """

    instances = []

    def __init__(self, bus=1):
        self.bus = bus
        self.writes = []        # (reg, list(data), time.time())
        self.read_calls = []    # (reg, length, time.time())
        self.regs = dict(DEFAULT_REGS)
        self.fail_next_reads = 0
        FakeSMBus.instances.append(self)

    @classmethod
    def reset(cls):
        cls.instances = []

    # ------------------------------------------------------------- writes

    def write_i2c_block_data(self, addr, reg, data):
        self.writes.append((reg, list(data), time.time()))

    def write_byte_data(self, addr, reg, data):
        self.writes.append((reg, [data], time.time()))

    def write_byte(self, addr, reg):
        self.writes.append((reg, [0], time.time()))

    # -------------------------------------------------------------- reads

    def read_i2c_block_data(self, addr, reg, length):
        self.read_calls.append((reg, length, time.time()))
        if self.fail_next_reads > 0:
            self.fail_next_reads -= 1
            raise IOError('injected I2C read failure (reg 0x%02X)' % reg)
        return list(self.regs.get(reg, [0] * length))[:length]

    def read_byte_data(self, addr, reg):
        data = self.read_i2c_block_data(addr, reg, 1)
        return data[0]

    # ----------------------------------------------------------- queries

    def motor_writes(self):
        """Only the motor register (0x01) writes: [[motor_id, dir, speed]]."""
        return [list(d) for reg, d, _ts in self.writes if reg == 0x01]


def spin(executor, seconds, step=0.05):
    """Drive one executor with spin_once for roughly ``seconds``."""
    end = time.time() + seconds
    while time.time() < end:
        executor.spin_once(timeout_sec=step)


def publish_until(executor, pub, msg, condition, timeout=3.0):
    """Re-publish ``msg`` and spin until ``condition()`` or timeout.

    Re-publishing is required because default Reliable QoS is volatile: a
    single publish before DDS discovery completes would be lost.
    """
    end = time.time() + timeout
    while time.time() < end:
        pub.publish(msg)
        executor.spin_once(timeout_sec=0.05)
        if condition():
            return True
    return False
