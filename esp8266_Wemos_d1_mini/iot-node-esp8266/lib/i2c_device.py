class I2C_device:

    def __init__(self, addr, i2c):
        self.addr = addr
        self.i2c = i2c

    def readU16(self, reg):
        b = self.i2c.readfrom_mem(self.addr, reg, 2)
        return (b[1] << 8) + b[0]

    def readS16(self,reg):
        b = self.i2c.readfrom_mem(self.addr, reg, 2)
        val = (b[1] << 8) + b[0]
        if val >= 32768:
            val = val - 65536
        return val

    def readU16LE(self,reg):
        return self.readU16(reg)

    def readS16LE(self, reg):
        return self.readS16(reg)

    def readU8(self, reg):
        b = self.i2c.readfrom_mem(self.addr, reg, 1)
        return b[0]

    def readS8(self, reg):
        b = self.i2c.readfrom_mem(self.addr, reg, 1)
        val = b[0]
        if val >= 128:
            val = val - 256
        return val

    def write8(self, reg, val):
        buf = bytearray(1)
        buf[0] = val
        self.i2c.writeto_mem(self.addr, reg, buf)
