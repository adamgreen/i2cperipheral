# Archived - October 27th, 2023
**This project is no longer under active development and requires changes to work on the latest versions of MicroPython. Someone who is well versed on .mpy development may be able to fork this project and get it working again in the future so I will leave it here for reference.**

# i2cperipheral - MicroPython I2C Peripheral Module for the RP2040
This ```i2cperipheral``` module provides the ```I2CPeripheral``` class used to implement a I2C peripheral (slave) on the RP2040 microcontroller using its built-in I2C hardware. It is built as a native MicroPython ```.mpy``` package and just needs to be installed in your device's ```sys.path``` ('/', '/lib', ...).

## class I2CPeripheral
Example usage:
```python
import i2cperipheral

i2c = i2cperipheral.I2CPeripheral(bus=0, sclPin=1, sdaPin=0, address=0x12)
data = bytearray(4)
while True:
    regAddressBuff = bytearray(1)

    # First thing master should send is register address.
    # Poll to see if it has been received yet.
    if not i2c.have_recv_req():
        continue
    i2c.recv(regAddressBuff, timeout=0)

    # Wait for master to send either the read or write.
    while (not i2c.have_recv_req()) and (not i2c.have_send_req()):
        pass

    # Only support read/write requests for register 0x01.
    regAddress = regAddressBuff[0]
    if regAddress != 0x01:
        # Handle invalid address.
        continue

    # Handle the master read/write request.
    if i2c.have_recv_req():
        i2c.recv(data, timeout=1000)
    else:
        i2c.send(data, timeout=1000)
```
An expanded example can be found in the [examples folder](examples/I2CSlave.py).

### Constructors
```class i2cperipheral.I2CPeripheral(bus, sclPin, sdaPin, *, address)```<br>
Construct and return a new I2CPeripheral object using the following parameters:
* *bus* identifies the I2C peripheral instance on the RP2040 to be used by this object. It can be 0 or 1.
* *sclPin* specifies the pin connected to the SCL signal. The RP2040 restricts which pins can be used for SCL on a particular bus.
* *sdaPin* specifies the pin connected to the SDA signal. The RP2040 restricts which pins can be used for SCL on a particular bus.
* *address* is an optional key word argument indicating the address to be assigned to this peripheral on the I2C bus. If it is specified then the I2C hardware will be initialized for use from the constructor. If not specified then the I2C hardware won't be initialized until later when the ```init()``` method is called.

Examples:
```python
# Will need to call the init() method later to finish setting up the peripheral.
i2c = i2cperipheral.I2CPeripheral(bus=0, sclPin=1, sdaPin=0)
```
```python
# The peripheral will be completely setup after this call.
i2c = i2cperipheral.I2CPeripheral(bus=0, sclPin=1, sdaPin=0, address=0x12)
```


### Init / Deinit Methods
```init(*, address=0x12)```<br>
Finish initializing the I2C peripheral hardware using the following parameters:
* *address* is an optional key word argument indicating the address to be assigned to this peripheral on the I2C bus. It will default to a value of 0x12.

Only needs to be called if the *address* argument wasn't specified in the constructor.

Examples:
```python
# The peripheral will listen on the I2C bus for requests to address 0x34.
i2c = i2cperipheral.I2CPeripheral(bus=0, sclPin=1, sdaPin=0)
i2c.init(0x34)
```
```python
# The peripheral will listen on the I2C bus for requests to the default address of 0x12
i2c = i2cperipheral.I2CPeripheral(bus=0, sclPin=1, sdaPin=0)
i2c.init()
```

```deinit()```<br>
Shutdown the I2C peripheral so that it no longer listens and responds to requests on the I2C bus.

Example:
```python
i2c.init()
# Shutdown the peripheral.
i2c.deinit()
```


### Request Status Methods
```have_recv_req()```<br>
Determine if the I2C controller (master) has made a write request of this peripheral that can be handled by calling the ```recv()``` method.

Example:
```python
buff = bytearray(1)
if i2c.have_recv_req():
    i2c.recv(buff)
```


```have_send_req()```<br>
Determine if the I2C controller (master) has made a read request of this peripheral that can be handled by calling the ```send()``` method.

Example:
```python
data = 25
if i2c.have_send_req():
    i2c.send(data)
```


### Data Transfer Methods
```recv(buffer, *, timeout=5000000)```<br>
Receives data from the controller (master) using the following parameters:
* *buffer* is the bytearray to receive bytes into. It can also be an integer indicating the amount of data to read into a bytearray allocated by recv().
* *timeout* is an optional parameter indicating the timeout in microseconds. It defaults to 5 seconds.

Returns: The bytearray into which the data was read or an OSError exception if the receive times out.

Examples:
```python
# Receive 1 byte of data into allocated array or will time out if the
# controller hasn't sent any data yet. It won't wait for new data to be sent.
buff = bytearray(1)
try:
    i2c.recv(buff, 0)
except OSError:
    print("Timed out")
```
```python
# Receive 2 bytes of data into recv() allocated array.
if i2c.have_recv_req():
    buff = i2c.recv(2)
```


```send(buffer, *, timeout=5000000)```<br>
Sends data to the controller (master) using the following parameters:
* *buffer* is the bytearray of bytes to send. It can also be an integer representing the value of the single byte to be sent.
* *timeout* is an optional parameter indicating the timeout in microseconds. It defaults to 5 seconds.

Returns: Will throw an OSError exception if the send times out or the transfer is aborted by the I2C hardware.

Examples:
```python
# Sends 2 bytes with a values of 0x5A and 0xA5. Will time out if the controller
# hasn't sent a read request yet.
buff = bytearray(2)
buff[0] = 0x5a
buff[1] = 0xa5
try:
    i2c.send(buff, 0)
except OSError:
    print("Timed out")
```
```python
# Sends a single byte with a value of of 0x11 to the controller if the controller
# has already sent a read request causing have_send_req() to return True.
if i2c.have_send_req():
    buff = i2c.send(2)
```



## How to Build
The latest ```i2cperipheral.mpy``` release can be found in the root of this repository. If you want to customize and build your own version then there are a few steps that you will need to complete. The following build steps were tested on macOS but should work on other *nix systems as well:
* ```make init``` - This does a few things to prepare for a full build of the library.
  * Makes sure that the required micropython related submodules are initialized and updated.
  * Applies patches to MicroPython's native .mpy build tools to support the ARMv6M architecture used by the RP2040 device.
  * Build MicroPython's mpy-cross tool for cross compiling i2cperipheral.py.
  * Pull the few RP2040 Pico SDK source files needed by this library into the root of this repository and patch them so that they just include the code required. The MicroPython tools don't automatically throw out unused code.
* ```pip install 'pyelftools>=0.25'``` - Install the Python library needed by mpy_ld.py to link together native object files into a .mpy library.
* ```make``` - Should now be able to build the i2cperipheral.mpy library.
