#new line in file
from smbus2 import SMBus
import time
import math
import more_itertools as mit


class CTRL1:
    ID = 0x20
    YEN = 0
    XEN = 1
    ZEN = 2
    PD = 3
    BW0 = 4
    BW1 = 5
    DR0 = 6
    DR1 = 7


class CTRL2:
    ID = 0x21
    HPCF0 = 0
    HPCF1 = 1
    HPCF2 = 2
    HPCF3 = 3
    HPM0 = 4
    HPM1 = 5
    LVLen = 6
    EXTRen = 7


class CTRL3:
    ID = 0x22
    INT2_Empty = 0
    INT2_ORun = 1
    INT2_FTH = 2
    INT2_DRDY = 3
    PP_OD = 4
    H_Lactive = 5
    INT1_Boot = 6
    INT1_IG = 7


class CTRL4:
    ID = 0x23
    SIM = 0
    ST1 = 1
    ST2 = 2
    IMPen = 3
    FS0 = 4
    FS1 = 5
    BLE = 6
    BDU = 7


class CTRL5:
    ID = 0x24
    Out_Sel0 = 0
    Out_Sel1 = 1
    IG_Sel0 = 2
    IG_Sel1 = 3
    HPen = 4
    StopOnFTH = 5
    FIFO_EN = 6
    BOOT = 7


class STATUS:
    ID = 0x27
    XDA = 0
    YDA = 1
    ZDA = 2
    ZYXDA = 3
    XOR = 4
    YOR = 5
    ZOR = 6
    ZYXOR = 7


class FIFO_CTRL:
    ID = 0x2e
    FTH0 = 0
    FTH1 = 1
    FTH2 = 2
    FTH3 = 3
    FTH4 = 4
    FM0 = 5
    FM1 = 6
    FM2 = 7


class FIFO_SRC:
    ID = 0x2f
    FSS0 = 0
    FSS1 = 1
    FSS2 = 2
    FSS3 = 3
    FSS4 = 4
    EMPTY = 5
    OVRN = 6
    FTH = 7


class DATARATE:
    DATARATE_800HZ_110 = 0b1111
    DATARATE_800HZ_50 = 0b1110
    DATARATE_800HZ_35 = 0b1101
    DATARATE_800HZ_30 = 0b1100
    DATARATE_400HZ_110 = 0b1011
    DATARATE_400HZ_50 = 0b1010
    DATARATE_400HZ_25 = 0b1001
    DATARATE_400HZ_20 = 0b1000
    DATARATE_200HZ_70 = 0b0111
    DATARATE_200HZ_50 = 0b0110
    DATARATE_200HZ_25 = 0b0101
    DATARATE_200HZ_12_5 = 0b0100
    DATARATE_100HZ_25 = 0b0001
    DATARATE_100HZ_12_5 = 0b0000


class MODE:
    BYPASS = 0 << FIFO_CTRL.FM0 | 0 << FIFO_CTRL.FM1 | 0 << FIFO_CTRL.FM2
    FIFO = 1 << FIFO_CTRL.FM0 | 0 << FIFO_CTRL.FM1 | 0 << FIFO_CTRL.FM2
    STREAM = 0 << FIFO_CTRL.FM0 | 1 << FIFO_CTRL.FM1 | 0 << FIFO_CTRL.FM2
    STREAM_TO_FIFO = 1 << FIFO_CTRL.FM0 | 1 << FIFO_CTRL.FM1 | 0 << FIFO_CTRL.FM2
    BYPASS_TO_STREAM = 0 << FIFO_CTRL.FM0 | 0 << FIFO_CTRL.FM1 | 1 << FIFO_CTRL.FM2
    DYNAMIC_STREAM = 0 << FIFO_CTRL.FM0 | 1 << FIFO_CTRL.FM1 | 1 << FIFO_CTRL.FM2
    BYPASS_TO_FIFO = 1 << FIFO_CTRL.FM0 | 1 << FIFO_CTRL.FM1 | 1 << FIFO_CTRL.FM2


class Vector:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z


class Gyr03b:
    OUT_X_L = 0x28
    OUT_X_H = 0x29
    OUT_Y_L = 0x2a
    OUT_Y_H = 0x2b
    OUT_Z_L = 0x2c
    OUT_Z_H = 0x2d

    def __init__(self, channel, device_id, mode=MODE.BYPASS, dps=2000, datarate=DATARATE.DATARATE_400HZ_50):
        self.channel = channel
        self.device_id = device_id
        self.bus = SMBus(self.channel)
        self.dps = dps
        if self.dps == 500:
            self.__write_or_byte(CTRL4.ID, 0x10)
            self.dps_per_digit = 0.0175
        elif self.dps == 2000:
            self.__write_or_byte(CTRL4.ID, 0x20)
            self.dps_per_digit = 0.07
        else:
            self.__write_or_byte(CTRL4.ID, 0x00)
            self.dps_per_digit = 0.00875
            self.dps = 250
        self.mode = mode
        self.set_mode(self.mode)
        self.enable_xyz()
        self.set_bandwidth_rate(datarate)
        self.bias = Vector()
        self.threshold = Vector()
        self.norm = Vector()
        self.gyro = Vector()
        self.dps = dps
        self.configure(100)

    def __read_byte(self, register):
        return self.bus.read_byte_data(self.device_id, register)

    def __read_bit(self, register, bit):
        register_value = self.__read_byte(register)
        mask = 1 << bit
        register_masked = register_value & mask
        return register_masked >> bit

    def __write_byte(self, register, value):
        self.bus.write_byte_data(self.device_id, register, value)

    def __write_or_byte(self, register, value):
        register_value = self.__read_byte(register)
        register_value |= value
        self.__write_byte(register, register_value)

    def enable_xyz(self):
        enable = 1 << CTRL1.XEN | 1 << CTRL1.YEN | 1 << CTRL1.ZEN | 1 << CTRL1.PD
        self.__write_or_byte(CTRL1.ID, enable)

    def set_bandwidth_rate(self, bandwidth_rate):
        self.__write_or_byte(CTRL1.ID, bandwidth_rate << 4)

    def set_mode(self, mode, thresh=0):
        threshold = thresh
        value = mode + threshold
        self.__write_byte(FIFO_CTRL.ID, value)

    def read_x(self):
        x_lsb = self.__read_byte(self.OUT_X_L)
        x_msb = self.__read_byte(self.OUT_X_H)
        if x_msb & 0x80:
            return -0xffff + (x_lsb + (x_msb << 8))
        return x_lsb + (x_msb << 8)

    def read_y(self):
        y_lsb = self.__read_byte(self.OUT_Y_L)
        y_msb = self.__read_byte(self.OUT_Y_H)
        if y_msb & 0x80:
            return -0xffff + (y_lsb + (y_msb << 8))
        return y_lsb + (y_msb << 8)

    def read_z(self):
        z_lsb = self.__read_byte(self.OUT_Z_L)
        z_msb = self.__read_byte(self.OUT_Z_H)
        if z_msb & 0x80:
            return -0xffff + (z_lsb + (z_msb << 8))
        return z_lsb + (z_msb << 8)

    def read_raw_xyz(self):
        g = Vector()
        g.x = self.read_x()
        g.y = self.read_y()
        g.z = self.read_z()
        return g

    def read_xyz(self):
        raw_xyz = self.read_raw_xyz()
        self.norm.x = (raw_xyz.x - self.bias.x) * self.dps_per_digit
        self.norm.y = (raw_xyz.y - self.bias.y) * self.dps_per_digit
        self.norm.z = (raw_xyz.z - self.bias.z) * self.dps_per_digit
        if abs(self.norm.x) < self.threshold.x:
            self.norm.x = 0
        if abs(self.norm.y) < self.threshold.y:
            self.norm.y = 0
        if abs(self.norm.z) < self.threshold.z:
            self.norm.z = 0
        return self.norm

    def configure(self, samples):
        x_sum, y_sum, z_sum = 0, 0, 0
        x_sigma, y_sigma, z_sigma = 0, 0, 0
        for sample in range(1, samples):
            raw_xyz = self.read_raw_xyz()
            x_sum += raw_xyz.x
            y_sum += raw_xyz.y
            z_sum += raw_xyz.z
            x_sigma += raw_xyz.x * raw_xyz.x
            y_sigma += raw_xyz.y * raw_xyz.y
            z_sigma += raw_xyz.z * raw_xyz.z
            time.sleep(0.005)
        self.bias.x = x_sum / samples
        self.bias.y = y_sum / samples
        self.bias.z = z_sum / samples
        self.threshold.x = 0.1 * math.sqrt((x_sigma / samples) - (self.bias.x * self.bias.x))
        self.threshold.y = 0.1 * math.sqrt((y_sigma / samples) - (self.bias.y * self.bias.y))
        self.threshold.z = 0.1 * math.sqrt((z_sigma / samples) - (self.bias.z * self.bias.z))

    def check_new_data(self):
        return self.__read_bit(STATUS.ID, STATUS.ZYXDA)

    def close(self):
        self.bus.close()


if __name__ == "__main__":
    pitch, roll, yaw = 0, 0, 0
    gyr = Gyr03b(1, 0x6b)
    cycle = 0.05
    start_time = 0
    sleep_time = 0
    while True:
        start_time = time.time()
        raw_data = gyr.read_xyz()
        pitch = pitch + cycle * raw_data.y
        roll = roll + cycle * raw_data.x
        yaw = yaw + cycle * raw_data.z
        print('roll: {} pitch: {} yaw: {}'.format(roll, pitch, yaw))
        time.sleep(cycle - (time.time() - start_time))
    gyr.close()


