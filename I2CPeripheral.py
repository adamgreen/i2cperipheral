"""
   Copyright 2021 Adam Green (https://github.com/adamgreen/)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
"""
import machine

# I2CPeripheral class - I2C Slave functionality for RP2040.
class I2CPeripheral:
    def __init__(self, bus, sclPin, sdaPin, *, address=0xff):
        sys_clk = machine.freq();
        if address == 0xff:
            self.i2c = CI2CPeripheral(bus, sclPin, sdaPin, sys_clk)
        else:
            self.i2c = CI2CPeripheral(bus, sclPin, sdaPin, sys_clk, address)

    def init(self, *, address=0x12):
        self.i2c.init(address)

    def deinit(self):
        self.i2c.deinit()

    def send(self, buffer, *, timeout=5000000):
        if isinstance(buffer, int):
            val = buffer
            buff = bytearray(1)
            buff[0] = val
            self.i2c.send(buff, timeout)
        else:
            self.i2c.send(buffer, timeout)

    def recv(self, buffer, *, timeout=5000000):
        if isinstance(buffer, int):
            size = buffer
            buff = bytearray(size)
            self.i2c.recv(buff, timeout)
            return buff
        else:
            self.i2c.recv(buffer, timeout)
            return buffer

    def have_recv_req(self):
        return self.i2c.have_recv_req()

    def have_send_req(self):
        return self.i2c.have_send_req()

    def __str__(self):
        print(self.i2c)
        return ""
