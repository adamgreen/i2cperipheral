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
// CI2CPeripheral class - The C API for I2C Slave functionality for RP2040.
#include "py/dynruntime.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "pico/timeout_helper.h"

// MicroPython object structure which represents an I2CPeripheral object.
typedef struct rp2040_i2cp_obj_t {
    mp_obj_base_t base;
    i2c_inst_t*   pI2C;
    mp_int_t      sclPin;
    mp_int_t      sdaPin;
    bool          isInit;
} rp2040_i2cp_obj_t;

// The I2CPeripheral objects are singletons, one for each of the RP2040's supported I2C peripherals.
rp2040_i2cp_obj_t rp2040_i2cp_obj[2];

// The sys_clk frequency passed in from MicroPython via the constructor.
uint32_t          g_sysClock;



// Forward Declarations.
static bool isValidSclPin(mp_int_t i2c, mp_int_t pin);
static bool isValidSdaPin(mp_int_t i2c, mp_int_t pin);
static mp_obj_t rp2040_i2cp_init_helper(rp2040_i2cp_obj_t *pSelf, size_t argCount, const mp_obj_t *pArgs);
static bool isI2CAddressValid(uint32_t address);
static bool isI2CAddress7Bit(uint32_t address);
static bool isI2CAddressReserved(uint32_t address);
static bool sendDataToMaster(i2c_inst_t *pI2C, const uint8_t *pSrc, size_t len, absolute_time_t stopTime);
static bool isReadRequestedByMaster(i2c_inst_t* pI2C);
static bool isTxFifoFull(i2c_inst_t* pI2C);
static bool recvFromMaster(i2c_inst_t* pI2C, uint8_t* pDest, size_t len, absolute_time_t stopTime);



// *********************************************************************************************************************
//  I2CPeripheral Class Methods
// *********************************************************************************************************************
// Method to display object contents if it is print(obj) in MicroPython code.
static void rp2040_i2cp_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    rp2040_i2cp_obj_t *pSelf = MP_OBJ_TO_PTR(self_in);
    i2c_inst_t* pI2C = pSelf->pI2C;
    uint32_t i2cIndex = i2c_hw_index(pI2C);

    if (!pSelf->isInit) {
        mp_printf(print, "I2CPeripheral(bus=%u, sclPin=%d, sdaPin=%d)", i2cIndex, pSelf->sclPin, pSelf->sdaPin);
    } else {
        mp_printf(print, "I2CPeripheral(bus=%u, sclPin=%d, sdaPin=%d, address=0x%02x)", i2cIndex, pSelf->sclPin, pSelf->sdaPin, pI2C->hw->sar);
    }
}

/// \classmethod \constructor(bus, sclPin, sdaPin, sys_clk, [address])
///
/// Construct an I2C object on the given bus.
/// `bus` can be 0 or 1.
/// 'sclPin' can be any SCL capable pin for this bus.
/// 'sdaPin' can be any SDA capable pin for this bus.
/// 'address' is optional address of SPI peripheral. Must be set in init()
/// later.
/// With no additional parameters, the I2C object is created but not
/// initialised (it has the settings from the last initialisation of
/// the bus, if any).  If extra arguments are given, the bus is initialised.
/// See `init` for parameters of initialisation.
static mp_obj_t rp2040_i2cp_make_new(const mp_obj_type_t *pType, size_t argCount, size_t keywordCount, const mp_obj_t *pArgs)
{
    // I2C bus index, sclPin, sdaPin, and sys_clk integer arguments are required.
    if (argCount < 4) {
        mp_raise_ValueError(MP_ERROR_TEXT("Too few params"));
    }
    mp_int_t i2cIndex = mp_obj_get_int(pArgs[0]);
    mp_int_t sclPin = mp_obj_get_int(pArgs[1]);
    mp_int_t sdaPin = mp_obj_get_int(pArgs[2]);
    g_sysClock = mp_obj_get_int(pArgs[3]);
    argCount -= 4;
    pArgs += 4;

    // Validate the argument values.
    if (i2cIndex < 0 || i2cIndex >= MP_ARRAY_SIZE(rp2040_i2cp_obj)) {
        mp_raise_ValueError(MP_ERROR_TEXT("Invalid I2C bus"));
    }
    if (!isValidSclPin(i2cIndex, sclPin)) {
        mp_raise_ValueError(MP_ERROR_TEXT("Invalid SCL pin"));
    }
    if (!isValidSdaPin(i2cIndex, sdaPin)) {
        mp_raise_ValueError(MP_ERROR_TEXT("Invalid SDA pin"));
    }

    rp2040_i2cp_obj_t* pI2CObject = &rp2040_i2cp_obj[i2cIndex];
    pI2CObject->isInit = false;
    pI2CObject->sclPin = sclPin;
    pI2CObject->sdaPin = sdaPin;

    // Actually init the object now if the slave address was specified as well.
    if (argCount > 0) {
        rp2040_i2cp_init_helper(pI2CObject, argCount, pArgs);
    }

    return MP_OBJ_FROM_PTR(pI2CObject);
}

// NOTE: The 2 following routines were copied from the RP2040 MicroPython I2C Controller driver.
// SDA/SCL on even/odd pins, I2C0/I2C1 on even/odd pairs of pins.
static bool isValidSclPin(mp_int_t i2c, mp_int_t pin)
{
    return ((pin & 1) == 1 && ((pin & 2) >> 1) == i2c);
}

static bool isValidSdaPin(mp_int_t i2c, mp_int_t pin)
{
    return ((pin & 1) == 0 && ((pin & 2) >> 1) == i2c);
}

static mp_obj_t rp2040_i2cp_init_(size_t n_args, const mp_obj_t *args, mp_map_t *kw_args)
{
    return rp2040_i2cp_init_helper(MP_OBJ_TO_PTR(args[0]), n_args - 1, args + 1);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(rp2040_i2cp_init_obj, 1, rp2040_i2cp_init_);

/// \method init(addr=0x12)
///
/// Initialise the I2C bus with the given positional parameters:
///
///   - `addr` is the optional 7-bit address (defaults to 0x12)
static mp_obj_t rp2040_i2cp_init_helper(rp2040_i2cp_obj_t *pSelf, size_t argCount, const mp_obj_t *pArgs)
{
    i2c_inst_t* pI2C = pSelf->pI2C;
    mp_int_t address = 0x12;

    if (argCount > 0) {
        address = mp_obj_get_int(pArgs[0]);
    }
    if (!isI2CAddressValid(address)) {
        mp_raise_ValueError(MP_ERROR_TEXT("Invalid I2C addr"));
    }

    const uint32_t frequency = 400000;
    i2c_init(pI2C, frequency);
    gpio_set_function(pSelf->sclPin, GPIO_FUNC_I2C);
    gpio_set_function(pSelf->sdaPin, GPIO_FUNC_I2C);
    gpio_set_pulls(pSelf->sclPin, true, 0);
    gpio_set_pulls(pSelf->sdaPin, true, 0);


    i2c_set_slave_mode(pI2C, true, address);
    pSelf->isInit = true;
    return mp_const_none;
}

static bool isI2CAddressValid(uint32_t address)
{
    return (isI2CAddress7Bit(address) && !isI2CAddressReserved(address));
}

static bool isI2CAddress7Bit(uint32_t address)
{
    return (address & 0x7F) == address;
}

// NOTE: Copied from PicoSDK I2C driver.
static bool isI2CAddressReserved(uint32_t address)
{
    // Addresses of the form 000 0xxx or 111 1xxx are reserved. No slave should
    // have these addresses.
    return (address & 0x78) == 0 || (address & 0x78) == 0x78;
}

/// \method deinit()
/// Turn off the I2C bus.
static mp_obj_t rp2040_i2cp_deinit(mp_obj_t self)
{
    rp2040_i2cp_obj_t *pSelf = MP_OBJ_TO_PTR(self);
    i2c_deinit(pSelf->pI2C);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(rp2040_i2cp_deinit_obj, rp2040_i2cp_deinit);

/// \method send(send, timeout=5000000)
/// Send data on the bus:
///
///   - `send` is the data (buffer object) to send.
///   - `timeout` is the timeout in microseconds to wait for the send
///
/// Return value: `None`.
static mp_obj_t rp2040_i2cp_send(size_t argCount, const mp_obj_t *pArgs, mp_map_t *pKwArgs)
{
    if (argCount < 3) {
        mp_raise_ValueError(MP_ERROR_TEXT("Too few params"));
    }

    rp2040_i2cp_obj_t *pSelf = MP_OBJ_TO_PTR(pArgs[0]);
    i2c_inst_t*       pI2C = pSelf->pI2C;
    mp_obj_t          sendBuff = pArgs[1];
    mp_int_t          timeout = mp_obj_get_int(pArgs[2]);

    // Get the buffer to send from.
    mp_buffer_info_t buffInfo;
    mp_get_buffer_raise(sendBuff, &buffInfo, MP_BUFFER_READ);

    absolute_time_t stopTime = make_timeout_time_us(timeout);
    if (!sendDataToMaster(pI2C, buffInfo.buf, buffInfo.len, stopTime)) {
        mp_raise_OSError(MP_ETIMEDOUT);
    }

    // Make sure that the read request interrupt has been cleared.
    pI2C->hw->clr_rd_req;

    // Handle a transmit abort.
    uint32_t abortReason = pI2C->hw->tx_abrt_source;
    if (abortReason != 0) {
        // Clear the transmit abort to revive the I2C peripheral and then raise a Python exception.
        pI2C->hw->clr_tx_abrt;
        mp_raise_OSError(MP_EIO);
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2040_i2cp_send_obj, 1, rp2040_i2cp_send);

static inline bool sendDataToMaster(i2c_inst_t *pI2C, const uint8_t *pSrc, size_t len, absolute_time_t stopTime)
{
    for (size_t i = 0; i < len; ++i) {
        while (!isReadRequestedByMaster(pI2C) || isTxFifoFull(pI2C)) {
            if (time_reached(stopTime)) {
                return false;
            }
        }
        i2c_get_hw(pI2C)->data_cmd = *pSrc++;
    }
    return true;
}

static bool isReadRequestedByMaster(i2c_inst_t* pI2C)
{
    return ((pI2C->hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_RD_REQ_BITS) != 0);
}

static bool isTxFifoFull(i2c_inst_t* pI2C)
{
    const size_t IC_TX_BUFFER_DEPTH = 16;
    return (i2c_get_hw(pI2C)->txflr == IC_TX_BUFFER_DEPTH);
}

/// \method recv(recv, addr=0x00, timeout=5000)
///
/// Receive data on the bus:
///
///   - `recv` is a mutable buffer, which will be filled with received bytes
///   - `timeout` is the timeout in microseconds to wait for the receive
///
/// Return value: if `recv` is an integer then a new buffer of the bytes received,
/// otherwise the same buffer that was passed in to `recv`.
STATIC mp_obj_t rp2040_i2cp_recv(size_t argCount, const mp_obj_t *pArgs, mp_map_t *pKwArgs)
{
    if (argCount < 3) {
        mp_raise_ValueError(MP_ERROR_TEXT("Too few params"));
    }

    rp2040_i2cp_obj_t *pSelf = MP_OBJ_TO_PTR(pArgs[0]);
    mp_obj_t      recvBuff = pArgs[1];
    mp_int_t      timeout = mp_obj_get_int(pArgs[2]);

    // Get the buffer to receive into.
    mp_buffer_info_t buffInfo;
    mp_get_buffer_raise(recvBuff, &buffInfo, MP_BUFFER_WRITE);

    absolute_time_t stopTime = make_timeout_time_us(timeout);
    if (!recvFromMaster(pSelf->pI2C, buffInfo.buf, buffInfo.len, stopTime)) {
        mp_raise_OSError(MP_ETIMEDOUT);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2040_i2cp_recv_obj, 1, rp2040_i2cp_recv);

static bool recvFromMaster(i2c_inst_t* pI2C, uint8_t* pDest, size_t len, absolute_time_t stopTime)
{
    for (size_t i = 0; i < len; ++i) {
        while (!i2c_get_read_available(pI2C)) {
            if (time_reached(stopTime)) {
                return false;
            }
        }
        *pDest++ = (uint8_t)i2c_get_hw(pI2C)->data_cmd;
    }
    return true;
}

/// \method have_send_req()
///
/// Has the I2C Master sent a read request?
/// Return value: True or False
STATIC mp_obj_t rp2040_i2cp_have_send_req(size_t argCount, const mp_obj_t *pArgs, mp_map_t *pKwArgs)
{
    rp2040_i2cp_obj_t* pSelf = MP_OBJ_TO_PTR(pArgs[0]);
    return mp_obj_new_bool(isReadRequestedByMaster(pSelf->pI2C));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2040_i2cp_have_send_req_obj, 1, rp2040_i2cp_have_send_req);

/// \method have_recv_req()
///
/// Has the I2C Master sent a write request?
/// Return value: True or False
STATIC mp_obj_t rp2040_i2cp_have_recv_req(size_t argCount, const mp_obj_t *pArgs, mp_map_t *pKwArgs)
{
    rp2040_i2cp_obj_t* pSelf = MP_OBJ_TO_PTR(pArgs[0]);
    return mp_obj_new_bool(i2c_get_read_available(pSelf->pI2C) > 0);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2040_i2cp_have_recv_req_obj, 1, rp2040_i2cp_have_recv_req);




// *********************************************************************************************************************
//  mpy_init() is called by MicroPython when this module is imported.
//  Used to setup the class and present it to MicroPython.
// *********************************************************************************************************************
mp_obj_type_t rp2040_i2cp_type;
mp_map_elem_t rp2040_i2cp_locals_dict_table[6];
STATIC MP_DEFINE_CONST_DICT(rp2040_i2cp_locals_dict, rp2040_i2cp_locals_dict_table);

mp_obj_t mpy_init(mp_obj_fun_bc_t *self, size_t n_args, size_t n_kw, mp_obj_t *args)
{
    MP_DYNRUNTIME_INIT_ENTRY

    // Instance Methods
    rp2040_i2cp_locals_dict_table[0] = (mp_map_elem_t){ MP_OBJ_NEW_QSTR(MP_QSTR_init), MP_OBJ_FROM_PTR(&rp2040_i2cp_init_obj) };
    rp2040_i2cp_locals_dict_table[1] = (mp_map_elem_t){ MP_OBJ_NEW_QSTR(MP_QSTR_deinit), MP_OBJ_FROM_PTR(&rp2040_i2cp_deinit_obj) };
    rp2040_i2cp_locals_dict_table[2] = (mp_map_elem_t){ MP_OBJ_NEW_QSTR(MP_QSTR_send), MP_OBJ_FROM_PTR(&rp2040_i2cp_send_obj) };
    rp2040_i2cp_locals_dict_table[3] = (mp_map_elem_t){ MP_OBJ_NEW_QSTR(MP_QSTR_recv), MP_OBJ_FROM_PTR(&rp2040_i2cp_recv_obj) };
    rp2040_i2cp_locals_dict_table[4] = (mp_map_elem_t){ MP_OBJ_NEW_QSTR(MP_QSTR_have_recv_req), MP_OBJ_FROM_PTR(&rp2040_i2cp_have_recv_req_obj) };
    rp2040_i2cp_locals_dict_table[5] = (mp_map_elem_t){ MP_OBJ_NEW_QSTR(MP_QSTR_have_send_req), MP_OBJ_FROM_PTR(&rp2040_i2cp_have_send_req_obj) };

    rp2040_i2cp_type.base.type = mp_fun_table.type_type;
    rp2040_i2cp_type.name = MP_QSTR_CI2CPeripheral;
    rp2040_i2cp_type.print = rp2040_i2cp_print;
    rp2040_i2cp_type.make_new = rp2040_i2cp_make_new;
    rp2040_i2cp_type.locals_dict = (void*)&rp2040_i2cp_locals_dict;

    mp_store_global(MP_QSTR___name__, MP_OBJ_NEW_QSTR(MP_QSTR_i2cperipheral));
    mp_store_global(MP_QSTR_CI2CPeripheral, MP_OBJ_FROM_PTR(&rp2040_i2cp_type));

    // Initialize I2C instances that would normally be in .data section.
    i2c0_inst = (i2c_inst_t){i2c0_hw, false};
    i2c1_inst = (i2c_inst_t){i2c1_hw, false};
    rp2040_i2cp_obj[0] = (rp2040_i2cp_obj_t){{&rp2040_i2cp_type}, &i2c0_inst};
    rp2040_i2cp_obj[1] = (rp2040_i2cp_obj_t){{&rp2040_i2cp_type}, &i2c1_inst};

    MP_DYNRUNTIME_INIT_EXIT
}






// *********************************************************************************************************************
//  Functions called from other C code included in this project.
// *********************************************************************************************************************
void* memset(void *s, int c, size_t n)
{
    return mp_fun_table.memset_(s, c, n);
}

// Might be able to get from Python code and pass it down.
uint32_t clock_get_hz(enum clock_index clk_index)
{
    if (clk_index != clk_sys) {
        mp_raise_ValueError(MP_ERROR_TEXT("Unknown clock"));
    }
    return g_sysClock;
}

// Pulled in from Pico SDK's timer.c
uint64_t time_us_64()
{
    // Need to make sure that the upper 32 bits of the timer
    // don't change, so read that first
    uint32_t hi = timer_hw->timerawh;
    uint32_t lo;
    do {
        // Read the lower 32 bits
        lo = timer_hw->timerawl;
        // Now read the upper 32 bits again and
        // check that it hasn't incremented. If it has loop around
        // and read the lower 32 bits again to get an accurate value
        uint32_t next_hi = timer_hw->timerawh;
        if (hi == next_hi) break;
        hi = next_hi;
    } while (true);
    return ((uint64_t) hi << 32u) | lo;
}

// Pulled in from Pico SDK's timeout_helper.c
static bool check_single_timeout_us(timeout_state_t *ts)
{
    return time_reached(ts->next_timeout);
}

check_timeout_fn init_single_timeout_until(timeout_state_t *ts, absolute_time_t target)
{
    ts->next_timeout = target;
    return check_single_timeout_us;
}

// UNDONE: Pull source files from Pico SDK and place in folder here and then update to comment out code we don't use to shrink library.
