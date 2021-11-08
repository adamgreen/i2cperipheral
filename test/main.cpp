/* Copyright 2021 Adam Green (https://github.com/adamgreen/)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
// Tests the i2cperipheral library running on a RP2040 by sending it data over
// I2C and then reading it back to make sure that it round trips accurately.
#include <mbed.h>

int main()
{
    static Serial serial(USBTX, USBRX);
    static I2C i2c(p28, p27);
    const char wordReg = 0x01;
    const uint8_t address = 0x12 << 1;
    uint32_t counter = 0;
    const uint32_t delay = 0;

    serial.baud(230400);
    i2c.frequency(100000);
    while(1)
    {
        int result;

        counter++;
        wait_ms(delay);

        // Send current value of counter to peripheral.
        char writeCmd[1+sizeof(counter)];
        writeCmd[0] = wordReg;
        memcpy(writeCmd+1, &counter, sizeof(counter));
        result = i2c.write(address, writeCmd, sizeof(writeCmd), false);
        if (result)
        {
            printf("Write NAK\n");
            continue;
        }
        wait_ms(delay);

        // Read back value of counter and see if it matches value just written.
        const char readCmd[] = { wordReg };
        result = i2c.write(address, readCmd, sizeof(readCmd), true);
        if (result)
        {
            printf("Read Command NAK\n");
            continue;
        }
        uint32_t readCounter = 0;
        result = i2c.read(address, (char*)&readCounter, sizeof(readCounter), false);
        if (result)
        {
            printf("Read Bytes NAK\n");
            continue;
        }

        if (counter == readCounter)
        {
            printf("%lu\n", counter);
        }
        else
        {
            printf("Expected:%08lX Actual:%08lX\n", counter, readCounter);
        }
    }
}
