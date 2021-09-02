# rfEasyLinkRx
---

### SysConfig Notice

All examples will soon be supported by SysConfig, a tool that will help you 
graphically configure your software components. A preview is available today in 
the examples/syscfg_preview directory. Starting in 3Q 2019, with SDK version 
3.30, only SysConfig-enabled versions of examples will be provided. For more 
information, click [here](http://www.ti.com/sysconfignotice).

-------------------------

Project Setup using the System Configuration Tool (SysConfig)
-------------------------
The purpose of SysConfig is to provide an easy to use interface for configuring 
drivers, RF stacks, and more. The .syscfg file provided with each example 
project has been configured and tested for that project. Changes to the .syscfg 
file may alter the behavior of the example away from default. Some parameters 
configured in SysConfig may require the use of specific APIs or additional 
modifications in the application source code. More information can be found in 
SysConfig by hovering over a configurable and clicking the question mark (?) 
next to it's name.

### EasyLink Stack Configuration
Many parameters of the EasyLink stack can be configured using SysConfig 
including RX, TX, Radio, and Advanced settings. More information can be found in 
SysConfig by hovering over a configurable and clicking the question mark (?) 
next to it's name. Alternatively, refer to the System Configuration Tool 
(SysConfig) section of the Proprietary RF User's guide found in 
&lt;SDK_INSTALL_DIR&gt;/docs/proprietary-rf/proprietary-rf-users-guide.html. 

Example Summary
---------------
Example includes the EasyLink API and uses it to configure the RF driver to RX
packets. This project should be run in conjunction with the rfEasyLinkTx
project.

For more information on the EasyLink API and usage refer to the [Proprietary RF User's guide](http://dev.ti.com/tirex/#/?link=Software%2FSimpleLink%20CC13x2%2026x2%20SDK%2FDocuments%2FProprietary%20RF%2FProprietary%20RF%20User's%20Guide)

Peripherals Exercised
---------------------
* `Board_PIN_LED1` - Indicates an abort which is expected to happen 300 ms after RX has been scheduled
* `Board_PIN_LED2` - Indicates that a packet has been received
`Board_PIN_LED1` & `Board_PIN_LED2` indicate an error
* `Board_GPTIMER0A` - Used to timeout the receive operation if it runs over
300ms in the no-RTOS asynchronous implementation
    
Resources & Jumper Settings
---------------------------
> If you're using an IDE (such as CCS or IAR), please refer to Board.html in your project
directory for resources used and board-specific jumper settings. Otherwise, you can find
Board.html in the directory &lt;SDK_INSTALL_DIR&gt;/source/ti/boards/&lt;BOARD&gt;.

Board Specific Settings
-----------------------
1. The default frequency is:
    - 433.92 MHz for the CC1350-LAUNCHXL-433
    - 433.92/490 MHz for the CC1352P-4-LAUNCHXL
    - 2440 MHz on the CC2640R2-LAUNCHXL
    - 868.0 MHz for other launchpads
In order to change frequency, modify the smartrf_settings.c file. This can be
done using the code export feature in Smart RF Studio, or directly in the file
2. On the CC1352P1 the high PA is enabled (high output power) for all
Sub-1 GHz modes by default.
3. On the CC1352P-2 the high PA operation for Sub-1 GHz modes is not supported
4. On the CC1352P-4 the high PA is enabled (high output power) for all
Sub-1 GHz modes by default.
    - The center frequency for 2-GFSK is set to 490 MHz
    - **CAUTION:** The center frequency for SimpleLink long range (SLR) is set to 433.92 MHz,
    but the high output power violates the maximum power output requirement
    for this band
5. The CC2640R2 is setup to run all proprietary physical modes at a center
frequency of 2440 MHz, at a data rate of 250 Kbps

Example Usage
-------------
Run the example. Board_PIN_LED2 will toggle indicating a packet has been received.
Board_PIN_LED1 will toggle indicating an abort, which is expected to happen if a
packet is not received within 300 ms of the RX being scheduled. Board_PIN_LED1 and
Board_PIN_LED2 indicates an error (not expected to happen under normal conditions,
but may occur if there is some interference).

After running this application you should start the rfEasyLinkTx on a second
board to transmit packets. Until the rfEasyLinkTx has been started the
receiver will be continuously RX.

Application Design Details
--------------------------
This example shows how to use the EasyLink API to access the RF drive, set the
frequency and receive packets. The board will blink LED2 when a packet has been
received. When a second board is running rfEasyLinkTx example then the expected
behavior is that Board_PIN_LED2 will blink every 100 ms 10 times, the TX will then
wait for 300 ms before transmitting the next packets and if RFEASYLINKRX_ASYNC
is defined (as it is by default) then this will cause the rfEasyLinkRx example
to timeout and exercise the EasyLink_abort API. When an RX has been aborted
Board_PIN_LED1 should toggle. The rfEasyLinkTx board will then transmit another
burst of packets and Board_PIN_LED2 should blink another 10 times and the cycle
should repeat.

If RFEASYLINKRX_ADDR_FILTER is defined (as it is by default) then the RX
address filter will be enabled for address 0xaa. This will cause the
rfEasyLinkRx to only accept packets with a destination address of 0xaa, which
the rfEasyLinkTx example transmits. When using the rfEasyLinkTx example there
will be no difference in behavior when defining/un-defining
RFEASYLINKRX_ADDR_FILTER as the rfEasyLinkTx example will use a destination
address of 0xaa. However, if transmitting from another source like SmartRF
Studio and not using an address of 0xaa, then defining RFEASYLINKRX_ADDR_FILTER
will result in packets not being received.

A single task, "rfEasyLinkRxFnx", configures the RF driver through the EasyLink
API and receives messages.

EasyLink API
-------------------------
### Overview
The EasyLink API should be used in application code. The EasyLink API is
intended to abstract the RF Driver in order to give a simple API for
customers to use as is or extend to suit their application[Use Cases]
(@ref USE_CASES).

### General Behavior
Before using the EasyLink API:

  - The EasyLink Layer is initialized by calling EasyLink_init(). This
    initializes and opens the RF driver and configures a modulation scheme
    passed to EasyLink_init.
  - The RX and TX can operate independently of each other.

The following is true for receive operation:

  - RX is enabled by calling EasyLink_receive() or EasyLink_receiveAsync()
  - Entering RX can be immediate or scheduled
  - EasyLink_receive() is blocking and EasyLink_receiveAsync() is non-blocking
  - The EasyLink API does not queue messages so calling another API function
    while in EasyLink_receiveAsync() will return EasyLink_Status_Busy_Error
  - An Async operation can be canceled with EasyLink_abort()

The following apply for transmit operation:

  - TX is enabled by calling EasyLink_transmit(), EasyLink_transmitAsync()
    or EasyLink_transmitCcaAsync()
  - TX can be immediate or scheduled
  - EasyLink_transmit() is blocking and EasyLink_transmitAsync(),
    EasyLink_transmitCcaAsync() are non-blocking
  - EasyLink_transmit() for a scheduled command, or if TX cannot start
  - The EasyLink API does not queue messages so calling another API function
    while in either EasyLink_transmitAsync() or EasyLink_transmitCcaAsync()
    will return EasyLink_Status_Busy_Error
  - An Async operation can be canceled with EasyLink_abort()

### Error Handling
The EasyLink API will return EasyLink_Status containing success or error
  code. The EasyLink_Status codes are:

   - EasyLink_Status_Success
   - EasyLink_Status_Config_Error
   - EasyLink_Status_Param_Error
   - EasyLink_Status_Mem_Error
   - EasyLink_Status_Cmd_Error
   - EasyLink_Status_Tx_Error
   - EasyLink_Status_Rx_Error
   - EasyLink_Status_Rx_Timeout
   - EasyLink_Status_Busy_Error
   - EasyLink_Status_Aborted

### Power Management
The power management framework will try to put the device into the most
power efficient mode whenever possible. Please see the technical reference
manual for further details on each power mode.

The EasyLink Layer uses the power management offered by the RF driver Refer to the RF
Driver documentation for more details.

### No-RTOS Implementation
The No-RTOS implementation uses a general purpose timer to timeout the receive
operation, for the asynchronous case, if it exceeds 300ms.

### Supported Functions
    | Generic API function          | Description                                        |
    |-------------------------------|----------------------------------------------------|
    | EasyLink_init()               | Init's and opens the RF driver and configures the  |
    |                               | specified settings based on EasyLink_Params struct |
    | EasyLink_transmit()           | Blocking Transmit                                  |
    | EasyLink_transmitAsync()      | Non-blocking Transmit                              |
    | EasyLink_transmitCcaAsync()   | Non-blocking Transmit with Clear Channel Assessment|
    | EasyLink_receive()            | Blocking Receive                                   |
    | EasyLink_receiveAsync()       | Non-blocking Receive                                |
    | EasyLink_abort()              | Aborts a non blocking call                         |
    | EasyLink_enableRxAddrFilter() | Enables/Disables RX filtering on the Addr          |
    | EasyLink_getIeeeAddr()        | Gets the IEEE Address                              |
    | EasyLink_setFrequency()       | Sets the frequency                                 |
    | EasyLink_getFrequency()       | Gets the frequency                                 |
    | EasyLink_setRfPower()         | Sets the Tx Power                                  |
    | EasyLink_getRfPower()         | Gets the Tx Power                                  |
    | EasyLink_getRssi()            | Gets the RSSI                                      |
    | EasyLink_getAbsTime()         | Gets the absolute time in RAT ticks                |
    | EasyLink_setCtrl()            | Set RF parameters, test modes or EasyLink options  |
    | EasyLink_getCtrl()            | Get RF parameters or EasyLink options              |
    | EasyLink_getIeeeAddr()        | Gets the IEEE address                              |

### Frame Structure
The EasyLink implements a basic header for transmitting and receiving data. This header supports
addressing for a star or point-to-point network with acknowledgments.

Packet structure:

     _________________________________________________________
    |           |                   |                         |
    | 1B Length | 1-64b Dst Address |         Payload         |
    |___________|___________________|_________________________|


Note for IAR users: When using the CC1310DK, the TI XDS110v3 USB Emulator must
be selected. For the CC1310_LAUNCHXL, select TI XDS110 Emulator. In both cases,
select the cJTAG interface.
