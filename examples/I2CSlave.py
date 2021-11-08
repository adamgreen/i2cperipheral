import i2cperipheral
import utime

def ignore_bad_reg_address(i2c, regAddress):
    print("Unexpected register address: ", regAddress)
    if i2c.have_recv_req():
        print("recv")
        # Read in bytes and throw them away.
        dummyBuff = bytearray(1)
        done = False
        while not done:
            try:
                i2c.rev(dummyBuff, timeout=1000)
            except OSError:
                # Have finally recevied timeout so can wait
                # for next command now.
                done = True
    else:
        # Feed master zeroes until it is satisfied.
        dummyBuff = bytearray(1)
        dummyBuff[0] = 0x00;
        while i2c.have_send_req():
            i2c.send(dummyBuff, timeout=1000)
            utime.sleep_ms(1)

def poll_i2c(i2c, data):
    regAddressBuff = bytearray(1)

    # First thing master should send is register address.
    # Poll to see if it has been received yet.
    if not i2c.have_recv_req():
        return
    i2c.recv(regAddressBuff, timeout=0)
    
    # Wait for master to send either the read or write.
    while (not i2c.have_recv_req()) and (not i2c.have_send_req()):
        pass
    
    # Only support read/write requests for address 0x01.
    regAddress = regAddressBuff[0]
    if regAddress != 0x01:
        ignore_bad_reg_address(i2c, regAddress)
        return
    
    # Handle the master read/write request.
    if i2c.have_recv_req():
        i2c.recv(data, timeout=1000)
    else:
        i2c.send(data, timeout=1000)
    

i2c = i2cperipheral.I2CPeripheral(bus=0, sclPin=1, sdaPin=0, address=0x12)
data = bytearray(4)
while True:
    poll_i2c(i2c, data)
    
