##
## GPSDRO - Symmetricom Rubidium Oscillator Continious Discipliner
## Copyright (C) 2025 RandomThings00
##
##    This program is free software: you can redistribute it and/or modify
##    it under the terms of the GNU General Public License as published by
##    the Free Software Foundation, either version 3 of the License, or
##    (at your option) any later version.
##
##    This program is distributed in the hope that it will be useful,
##    but WITHOUT ANY WARRANTY; without even the implied warranty of
##    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
##    GNU General Public License for more details.
##
##    You should have received a copy of the GNU General Public License
##    along with this program.  If not, see <https://www.gnu.org/licenses/>.
##
##############################################################################
##    Github: https://github.com/randomthings00/gpsdro-symmetricom
##    EEVBlog Forum: https://www.eevblog.com/forum/metrology/symmetricom-x99-rubidium-oscillator/
##
import time
import struct
import gc
import os

#
# System Specific Data
#
SYSTEM_DATA         = os.uname()
STORAGE_DATA        = os.statvfs("/")
OVERCLOCK_SYS       = 0
STATUS_LED			= 0

if ( SYSTEM_DATA.sysname.find("Linux") >= 0 ):
    import serial
else:
    import machine

#
# Constants
#
RUBIDIUM			= 1
GPS					= 2

SYM_UNKNOWN	    	= 0
SYM_SA22C	    	= 22
SYM_X72		    	= 72
SYM_X99		    	= 99

POLL_PPS			= 1000

#
# Used for the dds disciplining and tracking
#
STATE_INITIAL		= 0
STATE_RUNNING		= 1
STATE_HOLDOVER		= 2
STATE_JAMSYNCH		= 3
STATE_CALCSLOPE		= 4
STATE_US_DISCIPLINE	= 5
STATE_S_DISCIPLINE	= 6
STATE_DISCIPLINE	= 7
STATE_UL_DISCIPLINE	= 8

STATE_POS_COUNTER	= 0
STATE_POS_DDS		= 1
STATE_POS_RCOUNT	= 2
STATE_POS_RPPS		= 3
STATE_POS_XX		= 4
STATE_POS_YY		= 5
STATE_POS_XY		= 6
STATE_POS_OLD_DDS	= 7
STATE_POS_TIMESTAMP	= 8

STATE_HOLD_START    = 0
STATE_HOLD_CURRENT  = 1
STATE_HOLD_SHORT    = 2
STATE_HOLD_LONGER   = 3
STATE_HOLD_FOREVER  = 4

STATE_HLD_COUNTER	= 0
STATE_HLD_DDS		= 1
STATE_HLD_OLD_DDS	= 2
STATE_HLD_PPS   	= 3
STATE_HLD_LAST_PPS  = 4
STATE_HLD_STATE     = 5
STATE_HLD_TIMESTAMP	= 6

HLD_STATE_OFF       = 0
HLD_STATE_START     = 1
HLD_STATE_RB_LOCK   = 2
HLD_STATE_RUN       = 3

PPS_ROLLOVER        = 864000000
PPS_TRIGGER         = 10000

DISCIPLINE_ENTRIES	= 2200
#DISCIPLINE_ENTRIES	= 0

#
# These values coded constants for doing calculations.
#
DDS_BASE_VALUE		= float(1e-11)
DDS_LOW_VALUE		= float(2e-12)
DDS_CAL_VALUE		= float(2e-11)
DDS_INCR_VALUE		= float(2e-1)


DDS_CHECK_INTERVAL	= 10
#
# Used for short term tweaking of the DDS
#
# PPS weighted - 10 second window, 1 sawtooth
# it creates a +/-20 ns variation using a GPS PPS
#DDS_DRIFT_LIMIT    = 1.05
#DDS_DRIFT_WINDOW   = 10
# Weighted against 6 sawtooth pattern
# Still testing this -- allows for better tracking
# and keeps it to about +/- 5ns..
DDS_DRIFT_LIMIT	    = 0.75
DDS_DRIFT_WINDOW    = 60

#
# Global Variables
#
symModelArray 			= {};
symStatusArray			= {};
symSumsArray			= [[int(0),float(0),float(0),float(0),float(0),float(0),float(0),float(0), int(0)],
                           [int(0),float(0),float(0),float(0),float(0),float(0),float(0),float(0), int(0)],
                           [int(0),float(0),float(0),float(0),float(0),float(0),float(0),float(0), int(0)],
                           [int(0),float(0),float(0),float(0),float(0),float(0),float(0),float(0), int(0)],
                           [int(0),float(0),float(0),float(0),float(0),float(0),float(0),float(0), int(0)],
                           [int(0),float(0),float(0),float(0),float(0),float(0),float(0),float(0), int(0)],
                           [int(0),float(0),float(0),float(0),float(0),float(0),float(0),float(0), int(0)],
                           [int(0),float(0),float(0),float(0),float(0),float(0),float(0),float(0), int(0)],
                           [int(0),float(0),float(0),float(0),float(0),float(0),float(0),float(0), int(0)]];
symHoldoverArray        = [ [int(0),float(0),float(0),int(0),int(0),int(0),int(0)],
                            [int(0),float(0),float(0),int(0),int(0),int(0),int(0)],
                            [int(0),float(0),float(0),int(0),int(0),int(0),int(0)],
                            [int(0),float(0),float(0),int(0),int(0),int(0),int(0)],
                            [int(0),float(0),float(0),int(0),int(0),int(0),int(0)] ];

bufferStr				= "";
bufferArray				= {};

#
# Trackers
#
POLL_HEALTH_OFFSET	= POLL_PPS/2
POLL_DDS_OFFSET		= POLL_PPS/4
PPS_AVG_TRK			= 10 * DDS_DRIFT_WINDOW

symPPS 					= [ int(0) ] * PPS_AVG_TRK;
symPos 					= 0;
symPPScounter 			= 0;

#
# ******** REMEMBER BASE IS 2e-11 ********
# so if it's a 1.234e-10, it's actually, 12.34 starting value
#
ddsAdjValue 			= float(0.0);
#ddsAdjValue 			= float(-74.2);
ddsAdjValueOld 			= float(0.0);

# Utility Functions

#
# Use to display platform specific information and storage details
# This has been tuned for the Raspberry Pi Pico and ESP32-S3
#
# Other platforms running micropython should be added here before
# running because it exits if it is not found!
#
def platform_setup():
    global rubidium, STATUS_LED, POLL_PPS
    
    osData = os.uname();
    storageSize = STORAGE_DATA[1] * STORAGE_DATA[2];
    storageFree = STORAGE_DATA[0] * STORAGE_DATA[3];
    storageUsed = storageSize - storageFree;
    KB = 1024
    MB = 1024 * 1024

    print (osData);
    print("Size : {:,} bytes, {:,} KB, {} MB".format(storageSize, storageSize / KB, storageSize / MB))
    print("Used : {:,} bytes, {:,} KB, {} MB".format(storageUsed, storageUsed / KB, storageUsed / MB))
    print("Free : {:,} bytes, {:,} KB, {} MB".format(storageFree, storageFree / KB, storageFree / MB))
    
    # Set up the UART, it's declared global once thsi is setup it's usable 
    # by all.
    #
    # Pico: UART 0, TX=0, RX=1
    # ESP32-S3 - Pins 43 and 44 are dedicated console out, in standalone 
    #     mode it doesn't work! Usingf UART1 locations for Pico
    #     but mapped to the BPI-Pico-S3
    # ESP-C3 - Pins 21 and 20 are teh dedicated console out
    # 
    # ESP32-S3: UART 1, TX=18, RX=16
    # ESP32-C3: UART 1, TX=6, RX=5
    #
    if (SYSTEM_DATA .nodename == "rp2" ):
        print ("Speed:", machine.freq());
        if (OVERCLOCK_SYS):
            print ("Overclocking to", 150000000);
            machine.freq(150000000);
        print ("Setting up UART for Raspberry Pi Pico..");
        rubidium = machine.UART(0, tx=machine.Pin(0), rx=machine.Pin(1), rxbuf=1024, timeout=200, timeout_char=10);
        STATUS_LED = machine.Pin(25, machine.Pin.OUT);
    elif ( SYSTEM_DATA.machine.find("ESP32S3") > 0 ):
        print ("Speed:", machine.freq());
        if (OVERCLOCK_SYS):
            print ("Overclocking to", 240000000);
            machine.freq(240000000);
        print ("Setting up UART for ESP32-S3..");
        rubidium = machine.UART(1, tx=machine.Pin(18), rx=machine.Pin(16), rxbuf=1024, timeout=200, timeout_char=10);
        STATUS_LED = machine.Pin(46, machine.Pin.OUT);
    elif ( SYSTEM_DATA.machine.find("ESP32C3") >= 0 ):
        print ("Speed:", machine.freq());
        if (OVERCLOCK_SYS):
            print ("Overclocking to", 160000000);
            machine.freq(160000000);
        print ("Setting up UART for ESP32-C3..");
        rubidium = machine.UART(1, tx=machine.Pin(0), rx=machine.Pin(1), rxbuf=1024, timeout=200, timeout_char=10);
        STATUS_LED = machine.Pin(8, machine.Pin.OUT);
    elif ( SYSTEM_DATA.sysname.find("Linux") >= 0 ):
        print ("Setting up UART for Linux..");
        rubidium = serial.Serial("/dev/ttyS4", timeout=0.2, inter_byte_timeout=0.001);        
        POLL_PPS = 1000000000
    else:
        print ("********** Platform not found aborting ************")
        exit(1);
    

#
# This pads the hex string to match input type and strips the "."
# that indicates it's a float:
#
#
def buffer_hex_float_output(hexString):
    # Remove trailing dot and make sure it's 8 hex digits
    hexString = hexString.strip('.')
    
    if len(hexString) != 8:
        return hexString  # skip if not a valid float hex
#        cLength = len(hexString);
#        tLenght = 8-(cLength % 8);
#        hexString = '0'*(tLenght) + hexString
    
    try:
        as_bytes = bytes.fromhex(hexString)
        return struct.unpack('>f', as_bytes)[0]  # big-endian float
    except:
        return hexString


#
#  This handles the abstraction of micropython time and regualr python time
#
def get_current_tick():
    if ( SYSTEM_DATA.sysname.find("Linux") >= 0 ):
        return time.monotonic_ns();
    else:
        return time.ticks_ms();


#
#  This handles the difference addition
#
def add_offset_to_current( offsetTick ):
    if ( SYSTEM_DATA.sysname.find("Linux") >= 0 ):
        return time.monotonic_ns() + offsetTick;
    else:
        return time.ticks_add(time.ticks_ms(), int(offsetTick));


#
#  This returns the difference
#
def diff_offset_to_current( offsetTick ):
    if ( SYSTEM_DATA.sysname.find("Linux") >= 0 ):
        return time.monotonic_ns() - offsetTick;
    else:
        return time.ticks_diff (time.ticks_ms(), int(offsetTick));


#
# Rollover adjustment on PPS based on how the Symmetricom units handle
# it via their internal crystal
#
def pps_rollover_correction( val ):

    if (val > ( int(symModelArray["CRYSTAL"]) / 2)):
        val = val - int(symModelArray["CRYSTAL"]);

    return val;


#
# Use to reset some constants for the internal oscillator
#
def redefine_constants():
    global DDS_CAL_VALUE, DDS_BASE_VALUE, DDS_LOW_VALUE, DDS_INCR_VALUE
    global POLL_HEALTH_OFFSET, POLL_DDS_OFFSET

    DDS_CAL_VALUE = 1 / int(symModelArray["CRYSTAL"]);
    DDS_INCR_VALUE = DDS_LOW_VALUE / DDS_BASE_VALUE;
    POLL_HEALTH_OFFSET	= POLL_PPS / 2;
    POLL_DDS_OFFSET		= POLL_PPS / 4;


#
# This gets the message from the serial buffer it's there to ensure the
# buffer is "empty" after each command.
#
# Note: to reduce fragmentation in the stack, this was moved up globally
#       so buffers are reallocated constantly in the functions.
#
def get_serial_data( type, bufParam ):
    global bufferStr, bufferArray
              
    bufferStr = "";

    if ( SYSTEM_DATA.sysname.find("Linux") >= 0 ):
        bufferStr  = rubidium.readlines();
        bufferArray = [line.decode('utf-8').strip().upper() for line in bufferStr]
    else:
        data = rubidium.read();
        try:
            bufferStr += data.decode('ascii').upper();
        except:
            bufferStr = "";
        bufferArray = bufferStr.split("\r\n");
          

#
# This sends the message to the serial port make sure you call
# get_serial_data after to ensure you flush the buffer.
#
def send_serial_data( type, bufParam ):
    global bufferStr
    
    if ( SYSTEM_DATA.sysname.find("Linux") >= 0 ):
        rubidium.write(bufParam.encode())
    else:
        rubidium.write(bufParam);


#
# Used to get Model details from the "i" command
#
def get_symmetricom_model():
    global symModelArray, bufferStr, bufferArray, rubidium

    # This block is used to test for the 2 known
    # baud rates
    if ( SYSTEM_DATA.sysname.find("Linux") >= 0 ):
        rubidium = serial.Serial("/dev/ttyS4", timeout=0.2, inter_byte_timeout=0.001, baudrate=9600, bytesize=8, parity=serial.PARITY_NONE, stopbits=1);
    else:
        rubidium.init(baudrate=9600, bits=8, parity=None, stop=1);
    send_serial_data(RUBIDIUM, "i");
    get_serial_data(RUBIDIUM, 0);
    
    # Sometimes the try and catch just doesn't work
    # and it needs to be done again..  Happens.. 
    if ( len(bufferStr) < 4 ):
        if ( SYSTEM_DATA.sysname.find("Linux") >= 0 ):
            rubidium = serial.Serial("/dev/ttyS4", timeout=0.2, inter_byte_timeout=0.001, baudrate=57600, bytesize=8, parity=serial.PARITY_NONE, stopbits=1);
        else:
            rubidium.init(baudrate=57600, bits=8, parity=None, stop=1);
            rubidium.flush();
        send_serial_data(RUBIDIUM, "i");
        get_serial_data(RUBIDIUM, 0);

    for x in bufferArray:
        if ( x.find("BY SYMMETRICOM,") >= 0 ):
            tempVal = x[:x.find("BY SYMMETRICOM,")];
            if (tempVal.find("22") >=0 ):
                tempType = SYM_SA22C;
            elif (tempVal.find("X 9") >= 0):
                tempType = SYM_X99;
            else:
                tempType = SYM_X72;
            symModelArray["MODELTEXT"] = tempVal.strip();
            symModelArray["MODELENUM"] = tempType;
        
        if ( (x.find("VERSION ") >= 0) and (x.find(" OF") >= 0) ):
            tempVal = x[x.find("VERSION ",)+8:x.find(" OF")];
            symModelArray["FRIMWARE"] = tempVal;

        if ( (x.find("SERIAL CODE IS ") >= 0) and (x.find("-H,") >= 0) ):
            tempVal = x[x.find("SERIAL CODE IS ",)+15:x.find("-H,")];
#            tempVal = tempVal;
#            if ( symModelArray["MODELENUM"] == SYM_X99 ):
#                tempVal = tempVal;
#            else:
#                tempVal = struct.unpack('!L', bytes.fromhex(tempVal))[0];
            symModelArray["S/N"] = str(tempVal)+"-H";


        if ( ( (x.find("CRYSTAL: ") >= 0) or (x.find("CTL REG: ") >= 0) ) and (x.find(", ") >= 0) ):
            items = x.split(", ");
            for y in items:
                if ( y.find(": ") >= 0):
                    element = y.split(": ");
                    
                    # Kludge for HZ strings, Symmetricom why?1?
                    if ((element[1].find("HZ") >= 0)):
                        y = element[1].split("HZ");
                        if (symModelArray["MODELENUM"] != SYM_X99 ):
                            y = y[0].split(".");
                        element[1] = y[0];

                    if ( element[1].find(".") >= 0):
                        try:
                            tempVal = buffer_hex_float_output(element[1]);
                        except:
                            tempVal = float(element[1]);
                    elif ( element[0].find("CTL REG") >= 0):
                        # It's always hex!
                        tempVal = int(element[1], 16);
                    elif ( ( symModelArray["MODELENUM"] != SYM_X99 ) and (all(c in '0123456789ABCDEFabcdef' for c in element[1])) ):
                        tempVal = int(element[1], 16);
                    else:
                        # That's right sometimes it transmagically morphs
                        # into a float -- Symmetricom why?!
                        try:
                            tempVal = int(element[1]);
                        except:
                            tempVal = float(element[1]);                    

                    symModelArray[(element[0])] = tempVal;

        if ( ( (x.find(" SRVC: ") >= 0) and (x.find("FC: ") >= 0) ) and (x.find(", ") >= 0) ):
            items = x.split(", ");
            for y in items:
                if ( y.find(": ") >= 0):
                    element = y.split(": ");
                    tempVal = (element[1]);
                    symModelArray[(element[0])] = tempVal;


#
# Used to get PPS Delta from the "j" command
#
def get_pps_delta( ):
    global symStatusArray, bufferStr, bufferArray
    
    send_serial_data(RUBIDIUM, 'j');
    
    get_serial_data(RUBIDIUM, 0);
    tempVal = 0;
    
    for x in bufferArray:
        if ( x.find("DELTA REG:") >= 0):
            element = x.split(": ");
            if ( symModelArray["MODELENUM"] == SYM_X99 ):
                tempVal = int(element[1]);
            else:
                tempVal = int(element[1], 16)
                
            symStatusArray["LAST1PPSDELTA"] = symStatusArray["1PPSDELTA"];
            symStatusArray["1PPSDELTA"] = tempVal;
            symStatusArray["DIFF1PPSDELTA"] = pps_rollover_correction(symStatusArray["LAST1PPSDELTA"]) - pps_rollover_correction(symStatusArray["1PPSDELTA"]);


#
# Used to get Health details from the "w" command
#
def get_status_message():
    global symStatusArray, bufferStr, bufferArray
    
    send_serial_data(RUBIDIUM, 'w');

    get_serial_data(RUBIDIUM, 0);
    tempVal = "";

    for x in bufferArray:
        if ( x.find(": ") >= 0):
            element = x.split(": ");

            if ( element[1].find(".") >= 0):
                try:
                    tempVal = buffer_hex_float_output(element[1]);
                except:
                    print(element[1]);
                    tempVal = float(element[1]);
            elif ( element[0].find("IFPGACTL") >= 0):
                # It's always hex!
                tempVal = int(element[1], 16);
            elif ( ( symModelArray["MODELENUM"] != SYM_X99 ) and (all(c in '0123456789ABCDEFabcdef' for c in element[1])) ):
                tempVal = int(element[1], 16);
            else:
                # That's right sometimes it transmagically morphs
                # into a float -- Symmetricom why?!
                try:
                    tempVal = int(element[1]);
                except:
                    tempVal = float(element[1]);                    

            symStatusArray[element[0].strip()] = tempVal;


#
# Used to reset the 1PPS Delta
#
def set_tic_message( value ):
    global bufferStr, bufferArray
    
    stringVal = "k"+str(value)
    
    send_serial_data(RUBIDIUM, stringVal);
    get_serial_data(RUBIDIUM, 0);


#
# Used to set the D for frequency disciplining
#
def set_dds_message( value ):
    global bufferStr, bufferArray
    
    stringVal = "f"+str(value)
    
    send_serial_data(RUBIDIUM, stringVal);
    get_serial_data(RUBIDIUM, 0);


#
# Used to get the control register
#
def get_control_reg_message():
    global bufferStr, bufferArray
       
    send_serial_data(RUBIDIUM, "p");
    get_serial_data(RUBIDIUM, 0);

    tempVal = 0;
    
    for x in bufferArray:
        if ( x.find("ONTROL REG:") >= 0):
            element = x.split(": ");
            if (element[0].find("ONTROL REG") >= 0):
                tempVal = int(element[1], 16);
                symStatusArray["IFPGACTL"] = tempVal;
            
            
#
# Used to set the the control register
#
def set_control_reg_message( value ):
    global bufferStr, bufferArray
    
    stringVal = "q"+str(value)

    send_serial_data(RUBIDIUM, stringVal);
    get_serial_data(RUBIDIUM, 0);


#
# Use to help calcualte a slope value for disciplining
#
# Credits: This was taken from Lady Heather from Mark Sims.
#
def pps_cal_list( whichArray, inVal, countVal ):
    global symSumsArray

    symSumsArray[whichArray][STATE_POS_COUNTER]	= int(countVal);
    symSumsArray[whichArray][STATE_POS_RCOUNT]	+= int(countVal);
    symSumsArray[whichArray][STATE_POS_RPPS]	+= int(inVal);
    symSumsArray[whichArray][STATE_POS_XX]	+= (countVal * countVal);
    symSumsArray[whichArray][STATE_POS_YY]	+= (inVal * inVal);
    symSumsArray[whichArray][STATE_POS_XY]	+= (inVal * countVal);


#
# Used to copy entire sections from one entry to another for summary array.
#
def shift_pps_cal_entry ( fromArray, toArray ):
    global symSumsArray
    
    symSumsArray[toArray] = symSumsArray[fromArray];


#
# Use to reset an entry for the summary array
#
def reset_pps_cal_entry ( whihcArray ):
    global symSumsArray
    
    symSumsArray[whihcArray] = [int(0),float(0),float(0),float(0),float(0),float(0),float(0),float(0), int(0)];
     
    
#
# Use to fill an detials for holdover Entry
#
def record_hold_details ( whihcArray, curTick, curDds, oldDds, hldState  ):
    global symHoldoverArray

    symHoldoverArray[whihcArray][STATE_HLD_COUNTER] = curTick;
    symHoldoverArray[whihcArray][STATE_HLD_DDS] = curDds;
    symHoldoverArray[whihcArray][STATE_HLD_OLD_DDS] = oldDds;
    symHoldoverArray[whihcArray][STATE_HLD_PPS] = symStatusArray["1PPSDELTA"];
    symHoldoverArray[whihcArray][STATE_HLD_LAST_PPS] = symStatusArray["LAST1PPSDELTA"];
    symHoldoverArray[whihcArray][STATE_HLD_STATE] = hldState;


#
# Used to copy entire sections from one entry to another for Holdover array.
#
def shift_hold_details ( fromArray, toArray ):
    global symHoldoverArray
    
    symHoldoverArray[toArray] = symHoldoverArray[fromArray];


#
# Use to reset an entry for the Holdover array
#
def reset_hold_details ( whihcArray ):
    global symHoldoverArray
    
    symHoldoverArray[whihcArray] = [int(0),float(0),float(0),int(0),int(0),int(0),int(0)];
  
  
#
# This is used to calculate the slope and the DDS
#
# Credits: This was taken from Lady Heather from Mark Sims.
# Looks like a variation be a Linear Extrapolation?
#
def return_slope ( whichArray ):
    global symSumsArray
    
    slope = float(0.0);

    # Oops we reset the array and got here, it happens?
    if ( symSumsArray[whichArray][STATE_POS_COUNTER] == 0 ):
        return slope;

    slopeX = (symSumsArray[whichArray][STATE_POS_XY] - ((symSumsArray[whichArray][STATE_POS_RCOUNT] * symSumsArray[whichArray][STATE_POS_RPPS]) / symSumsArray[whichArray][STATE_POS_COUNTER])) * symSumsArray[whichArray][STATE_POS_COUNTER];
    slopeY = (symSumsArray[whichArray][STATE_POS_YY] - ((symSumsArray[whichArray][STATE_POS_RPPS] * symSumsArray[whichArray][STATE_POS_RPPS]) / symSumsArray[whichArray][STATE_POS_COUNTER])) * symSumsArray[whichArray][STATE_POS_COUNTER];
    if (slopeX != 0):
        slope = ((slopeY / slopeX)*symSumsArray[whichArray][STATE_POS_COUNTER])/symSumsArray[whichArray][STATE_POS_COUNTER]/symModelArray["CRYSTAL"];
        
    return slope;


#
# This is a quick fucntion to return the sum of an array and accounts
# for an wrap
#
def return_sum_array( theArray, posNow, posInterval, numEntries ):

    if ( posNow-posInterval < 0 ):
        dSums = sum ( (theArray[((posNow-posInterval)%(numEntries)):] + theArray[:(posInterval-posNow)] ));
    else:
        dSums = sum (theArray[((posNow-posInterval)%(numEntries)):((posNow)%(numEntries))])
        
    return dSums


###
### Calculate slope
###
### This rountine is called at the start to determine the proper DDS value
### so provide more consistent behaviour without hard coding it or pre-tuning
### the Symmetricom.
###
### 
def calculate_slope( whichArray, inDdsValue, disciplineDuration ):
    global ddsAdjValue, DDS_CAL_VALUE, DDS_BASE_VALUE, DDS_LOW_VALUE, DDS_INCR_VALUE
    global symSumsArray, STATUS_LED



    get_pps_delta();
    get_status_message();
    if ( inDdsValue == 0.0 ):
        print ("Calculating a new DDS value ( 0.0 )..");
        ddsAdjValue = inDdsValue;
    else: 
        print ( "Calculating DDS value with offSet (", inDdsValue, ").." );
        ddsAdjValue = inDdsValue;
    
    
    print ("  Setting DDS to: ", ddsAdjValue);
    oldDdsAdjValue = ddsAdjValue;
    set_dds_message( ddsAdjValue );
    get_pps_delta();

#   Should be used to get a proper start with current delta as 0 so we can 
#   track drift properly. We could calcualte based on delta long term but 
#   have to test
#
    print ("  Resetting PPS Delta to: 0 ");
    set_tic_message( symStatusArray["1PPSDELTA"] );    
    print ("    Waiting for PPS reset.", end="");
    get_pps_delta();
    retryTracker = 1;
    while ( symStatusArray["1PPSDELTA"] != 0 ):
        print (".", end="");
        get_pps_delta();
        time.sleep(1);
        if ( SYSTEM_DATA.sysname.find("Linux") < 0 ):  
            STATUS_LED.toggle();
        retryTracker += 1;
        if (retryTracker > 10):
            set_tic_message( symStatusArray["1PPSDELTA"] );
            retryTracker = 1;
    print ("");
    
    startTracker = get_current_tick(); 
    loopTracker = 1;
    valueTracker = 0;
    valueCounter = int(0);
    printedStatus = 1;

    get_pps_delta();
    get_control_reg_message();
    print ("  Start disciplining (", disciplineDuration, ")..");
    startTracker =  add_offset_to_current(POLL_PPS);

    while ( (loopTracker) and (valueCounter < disciplineDuration) ):

        if ( diff_offset_to_current(startTracker) >= 0):
            get_pps_delta();
            get_control_reg_message();
            if ( SYSTEM_DATA.sysname.find("Linux") < 0 ):  
                STATUS_LED.toggle();
            pps_cal_list(whichArray, pps_rollover_correction(symStatusArray["1PPSDELTA"]), valueCounter);
            
            if ( symStatusArray["IFPGACTL"] & 0x0002 ):
                print ("    Rubidium lock bad, exiting..");
                loopTracker = 0;
            elif ( abs(symStatusArray["DIFF1PPSDELTA"]) >= PPS_TRIGGER ):
                print ("    PPS Spike / Loss detected, exiting..");
                loopTracker = 0;
            else:
                startTracker = add_offset_to_current(POLL_PPS);
                valueTracker += pps_rollover_correction(symStatusArray["1PPSDELTA"]);
                valueCounter = valueCounter + 1;
            printedStatus = 0;
        else:
            if ( ((valueCounter % 100) == 0) and (printedStatus == 0) ):
                if ( (valueCounter % 200) == 0 ):
                    totalAverage = valueTracker / valueCounter;
                    slope = return_slope( whichArray );
                    print ("    Current Count: ", valueCounter, "-- current Delta: ", symStatusArray["1PPSDELTA"], "(", pps_rollover_correction(symStatusArray["1PPSDELTA"]), ")","-- Total Values:", valueTracker, "-- Average: ", totalAverage, "-- Slope: {:.5e}".format(slope) );
                else :
                    print ("    Current Count: ", valueCounter, "-- current Delta: ", symStatusArray["1PPSDELTA"], "(", pps_rollover_correction(symStatusArray["1PPSDELTA"]), ")");
                printedStatus = 1;
            time.sleep(0.01); 
        
    if (valueCounter == disciplineDuration):
        if (disciplineDuration == 0):
            totalAverage = 0.0;
        else:
            totalAverage = valueTracker / disciplineDuration;
            
        slope = return_slope( whichArray );
        print ("  Total value: {:}  -- Entries: {:}  --Average: {:}  -- Slope: {:.5e}".format(valueTracker, disciplineDuration, totalAverage, slope) );
        
        # Convert this slope to entry compatible unit format for the command line
        slope = return_slope( whichArray ) / DDS_BASE_VALUE;
        ddsCheckVal = round (slope, 1);
        
        
        if ( (int(ddsCheckVal*10) % 2) == 1 ):
            ddsCheckVal = ddsCheckVal + 0.1;
            
        ddsNewVal = ddsAdjValue + ddsCheckVal;
        symSumsArray[whichArray][STATE_POS_COUNTER] = disciplineDuration;
        symSumsArray[whichArray][STATE_POS_OLD_DDS] = oldDdsAdjValue;
        symSumsArray[whichArray][STATE_POS_TIMESTAMP] = symPPScounter;

        if (abs(ddsCheckVal) > 0.2):
            print ("  Current DDS Value: ", ddsAdjValue, "  -- DDS Offset: ", ddsCheckVal, " -- New DDS Value: ", ddsNewVal);
            symSumsArray[whichArray][STATE_POS_DDS] = ddsNewVal;
            ddsAdjValue = ddsNewVal;
            
            print ("    Setting DDS to: ", ddsAdjValue);
            set_dds_message( ddsAdjValue );
        else:
            symSumsArray[whichArray][STATE_POS_DDS] = oldDdsAdjValue;
            print ("  The value of change is lower than 0.2 on the DDS, no change will be made..");
  
        get_pps_delta();
        print ("  Resetting PPS Delta to: 0 ");
        set_tic_message( symStatusArray["1PPSDELTA"] );    
        print ("    Waiting for PPS reset.", end="");
        get_pps_delta();
        retryTracker = 1;
        while ( symStatusArray["1PPSDELTA"] != 0 ):
            print (".", end="");
            get_pps_delta();
            time.sleep(1);
            if ( SYSTEM_DATA.sysname.find("Linux") < 0 ):  
                STATUS_LED.toggle();
            retryTracker += 1;
            if (retryTracker > 10):
                set_tic_message( symStatusArray["1PPSDELTA"] );
                retryTracker = 1;
        print ("");  
        
        return 0;
    else:
        print ("  Not enough data points something went wrong DDS Value not changed..");
        return 1;
            
            
#
# Used to track the values returned by the 1PPS Delta
# 
# Tracking 1 second and store in the PPS array
#
#
def track_pps_average( inValue ):
    value = pps_rollover_correction(inValue);
    
    global symPPS, symPos
    global symPPScounter, symStatusArray
    
    symPPS[symPos] = value - symStatusArray["PPSOFFSET"];
    lastPos = symPos;

    symPos = ( symPos + 1 ) % PPS_AVG_TRK;
    symPPScounter += 1;
    
    # Rollover handling
    if (symPPScounter == PPS_ROLLOVER): symPPScounter = 0;
    

#
# Used to addjust the DDS by 0.2 at a time
#
# I had code that tried to adjust for larger jumps but realized it
# was mostly noise so we need a way to handle it.
#                    
def dds_check_and_adjust():
    global symPPS, symPos
    global symPPScounter, ddsAdjValue, ddsAdjValueOld

    if (symPPScounter > DDS_CHECK_INTERVAL):
        
        ppsval = return_sum_array( symPPS, symPos, DDS_DRIFT_WINDOW, PPS_AVG_TRK );
                   
        if ( abs(ppsval)/DDS_DRIFT_WINDOW >= DDS_DRIFT_LIMIT):
            multval = round( ppsval/DDS_DRIFT_WINDOW/DDS_DRIFT_LIMIT, 0 );
 
# This code was removed if there is PPS spike it throws it off
# too much there's probably a better way to do this!
#
# Values should be whole values and no fractions!
#            if (abs(ppsval) > 1000):
#                multval = 100;
#            elif (abs(ppsval) > 100):
#                multval = 10;
#
            
            ddsAdjValueOld = ddsAdjValue;
            ddsAdjValue = round (ddsAdjValue + ( multval * DDS_INCR_VALUE), 1);
        
#            set_tic_message( symStatusArray["1PPSDELTA"] );
            symStatusArray["PPSOFFSET"] = (pps_rollover_correction(symStatusArray["1PPSDELTA"]));
            set_dds_message( ddsAdjValue );
            
            return 1;
        else:
            return 0;
    else:
        return 0;

#
# Maiu function -- it all runs here!
#
def main():
    global symPPS, symPPScounter, symPos
    global symStatusArray, symSumsArray

    # Enable Garbage Collection    
    gc.enable();
    
    # Obtain and intitialize all model information
    platform_setup();
    get_symmetricom_model();
    get_status_message();
    redefine_constants();
    set_control_reg_message(0);

    print ("Model:", symModelArray["MODELTEXT"], "-- S/N:", symModelArray["S/N"], "-- Service Pin:", symModelArray["SRVC"] ) ;
    
    # Inititalize
    symStatusArray["1PPSDELTA"] = 0;
    symStatusArray["PPSOFFSET"] = 0;

    if ( symStatusArray["IFPGACTL"] & 0x0002 ):
        print ("Rubidium lock is bad, wait until lock is good.",end="");
        while ( symStatusArray["IFPGACTL"] & 0x0002):
            print (".",end=""); 
            get_control_reg_message();
            time.sleep(2);
            if ( SYSTEM_DATA.sysname.find("Linux") < 0 ):  
                STATUS_LED.toggle();
        print ("");
    else:
        print ("Rubidium lock is good.");

    # Used to reset the tic counter
    # Reports this maybe an issue on larger value drifts.
    set_tic_message( "5FFFFFF90" );
#    set_tic_message( "90" );
    get_pps_delta();
    
    # Initial disciplining
    while ( calculate_slope(STATE_INITIAL, ddsAdjValue, DISCIPLINE_ENTRIES) ):
#    while ( calculate_slope(STATE_INITIAL, ddsAdjValue, 0) ):
        print ("Failed Slope calculation, restarting..");
        

    get_status_message();
    print ("-------------------------");
    print ("Model: ", symModelArray);
    print ("Health: ", symStatusArray);
    print ("-------------------------");
    print ("Starting Main Loop");
    timeTracker = get_current_tick();
    
    ppsTracker = add_offset_to_current(POLL_PPS);
    healthTracker = add_offset_to_current((5*POLL_PPS));
    ddsTracker = add_offset_to_current(POLL_DDS_OFFSET + (DDS_CHECK_INTERVAL*POLL_PPS));
    loopTracker = 1;
    holdOverState = HLD_STATE_OFF;
    holdOverTime = 0;

    #  This is the "tick" loop where all the tracking and processing occurs
    while loopTracker:

        # 1 PPS processing -- it drifts but placing it up here hopefully 
        # will reduce the drift
        if ( diff_offset_to_current( ppsTracker) > 0):
            ppsTracker = add_offset_to_current(POLL_PPS);
            get_pps_delta();
            if ( SYSTEM_DATA.sysname.find("Linux") < 0 ):  
                STATUS_LED.toggle();

#
# This is the rough holdover code I put in for now, there's more than can be
# done here this is just a place holder.
#
#
            if ( holdOverState  == HLD_STATE_OFF ):
                if ( symStatusArray["IFPGACTL"] & 0x0002 ):
                    print ("*** Rubidium lock bad, going into Holdover..");
                    record_hold_details ( STATE_HOLD_START, symPPScounter, ddsAdjValue, ddsAdjValueOld, HLD_STATE_OFF );
                    holdOverState = HLD_STATE_RB_LOCK;
                    holdOverTime = symPPScounter;
                    track_pps_average(0);
                    continue;
                elif ( abs(symStatusArray["DIFF1PPSDELTA"]) >= PPS_TRIGGER ):
                    print ("*** PPS Spike / Loss detected, going into Holdover....");
                    record_hold_details ( STATE_HOLD_START, symPPScounter, ddsAdjValue, ddsAdjValueOld, HLD_STATE_OFF );
                    holdOverState = HLD_STATE_START;
                    holdOverTime = symPPScounter;
                    track_pps_average(0);
                    continue;
            elif ( holdOverState == HLD_STATE_START ):
                if ( symStatusArray["IFPGACTL"] & 0x0002 ):
                    holdOverState = HLD_STATE_RB_LOCK;
                elif ( abs(symStatusArray["DIFF1PPSDELTA"]) >= PPS_TRIGGER ):
                    holdOverState = HLD_STATE_OFF;
                    reset_pps_cal_entry(STATE_CALCSLOPE);
                    reset_hold_details ( STATE_HOLD_START );
                
                track_pps_average(0);
                continue;
            elif ( holdOverState == HLD_STATE_RB_LOCK ):
                if ( not(symStatusArray["IFPGACTL"] & 0x0002) ):
                    print ("*** Rubidium lock is good, leaving Holdover....");
                    while ( calculate_slope(STATE_HOLDOVER, ddsAdjValue, DISCIPLINE_ENTRIES) ):
                        print ("Failed Slope calculation, restarting..");

                    reset_pps_cal_entry(STATE_CALCSLOPE);
                    reset_hold_details ( STATE_HOLD_START );

                    for retryTracker in range(DDS_DRIFT_WINDOW):
                        symPPS[symPos] = 0;
                        symPos = ( symPos + 1 ) % PPS_AVG_TRK;
                        
                    holdOverState = HLD_STATE_OFF;
                    holdOverTime = 0;
                continue;

            # Rollover handling 
            if( symSumsArray[STATE_RUNNING][STATE_POS_COUNTER]+1 > PPS_ROLLOVER):
                reset_pps_cal_entry(STATE_RUNNING);

            if( symSumsArray[STATE_CALCSLOPE][STATE_POS_COUNTER]+1 > PPS_ROLLOVER):
                reset_pps_cal_entry(STATE_CALCSLOPE);

            pps_cal_list(STATE_RUNNING, pps_rollover_correction(symStatusArray["1PPSDELTA"]), symSumsArray[STATE_RUNNING][STATE_POS_COUNTER]+1);
            pps_cal_list(STATE_CALCSLOPE, pps_rollover_correction(symStatusArray["1PPSDELTA"]), symSumsArray[STATE_CALCSLOPE][STATE_POS_COUNTER]+1);
            track_pps_average(symStatusArray["1PPSDELTA"]);

        #
        # This is where the DDS adjustment occurs - This uses the brute force 
        # method, we will see how well this works and may switch over to using 
        # the CALCSLOPE values in the long term
        #                    
        if ( diff_offset_to_current(ddsTracker) > 0 ):

#
# Don't adjust anything in holdover state
#
            if ( holdOverState ):
                ddsTracker = add_offset_to_current((DDS_DRIFT_WINDOW*2)*POLL_PPS);
                continue;
                
            if ( dds_check_and_adjust() ):
                ddsTracker = add_offset_to_current((DDS_DRIFT_WINDOW*2)*POLL_PPS);

                # We will consider any adjustment that exceeds 
                # DISCIPLINE_ENTRIES period as the last disciplined value. 
                if  ( (symSumsArray[STATE_CALCSLOPE][STATE_POS_COUNTER]) >= (DISCIPLINE_ENTRIES) ):
                    if  ( (symSumsArray[STATE_CALCSLOPE][STATE_POS_COUNTER]) >= (2*DISCIPLINE_ENTRIES) ):
                        shift_pps_cal_entry(STATE_CALCSLOPE,STATE_UL_DISCIPLINE);
                        symSumsArray[STATE_UL_DISCIPLINE][STATE_POS_DDS] = ddsAdjValue;
                        symSumsArray[STATE_UL_DISCIPLINE][STATE_POS_OLD_DDS] = ddsAdjValueOld;
                        symSumsArray[STATE_UL_DISCIPLINE][STATE_POS_TIMESTAMP] = symPPScounter;
                    else:
                        shift_pps_cal_entry(STATE_CALCSLOPE,STATE_DISCIPLINE);
                        symSumsArray[STATE_DISCIPLINE][STATE_POS_DDS] = ddsAdjValue;
                        symSumsArray[STATE_DISCIPLINE][STATE_POS_OLD_DDS] = ddsAdjValueOld;
                        symSumsArray[STATE_DISCIPLINE][STATE_POS_TIMESTAMP] = symPPScounter;

                # We will condier any adjustment that is shoter than 
                # DISCIPLINE_ENTRIES period as the last holdover value. 
                if  ( (symSumsArray[STATE_CALCSLOPE][STATE_POS_COUNTER]) < (DISCIPLINE_ENTRIES) ):
                    if  ( (symSumsArray[STATE_CALCSLOPE][STATE_POS_COUNTER]) < (DISCIPLINE_ENTRIES/2)):
                        shift_pps_cal_entry(STATE_CALCSLOPE, STATE_US_DISCIPLINE);
                        symSumsArray[STATE_US_DISCIPLINE][STATE_POS_DDS] = ddsAdjValue;
                        symSumsArray[STATE_US_DISCIPLINE][STATE_POS_OLD_DDS] = ddsAdjValueOld;
                        symSumsArray[STATE_US_DISCIPLINE][STATE_POS_TIMESTAMP] = symPPScounter;

                    else:
                        shift_pps_cal_entry(STATE_CALCSLOPE, STATE_S_DISCIPLINE);
                        symSumsArray[STATE_S_DISCIPLINE][STATE_POS_DDS] = ddsAdjValue;
                        symSumsArray[STATE_S_DISCIPLINE][STATE_POS_OLD_DDS] = ddsAdjValueOld;
                        symSumsArray[STATE_S_DISCIPLINE][STATE_POS_TIMESTAMP] = symPPScounter;

                reset_pps_cal_entry(STATE_CALCSLOPE);
            else:
                ddsTracker = add_offset_to_current(DDS_CHECK_INTERVAL*POLL_PPS);
                
        # Display stuff here lowest priority to everything and the small sleep
        # if nothing to do.
        elif ( diff_offset_to_current(healthTracker) > 0):
            get_status_message();
            healthTracker = add_offset_to_current(10*POLL_PPS);
            
            rblock = "Good";
            if (symStatusArray["IFPGACTL"] & 0x0002): rblock = "Bad";
            
            holdState = "False";
            if (holdOverState): holdState = "True";
            
            slopeR = return_slope( STATE_RUNNING );
            slopeC = return_slope( STATE_CALCSLOPE );
            dSums =  return_sum_array( symPPS, symPos, DDS_DRIFT_WINDOW, PPS_AVG_TRK );

# Model: SA22C               HoldOver: False         Service Pin: LOW
# S/N: 123456789-H           Crystal:     60000000   In Voltage: 14.88893
# Rb Lock: Good              Current Temp:   54.25   Lamp Voltage: 13.52071
# PPS Delta: 5999999 ( -1)   Adjusted:     4 ( -5)   Counter: 383624
# R Slope: R(  5.3234e-12)   P.Hours:       123456   P.Ticks: 12345678
# C Slope: R( -2.2213e-11)   DDS Adjust:     -10.2   Averages: 1.0e-11

            if ( SYSTEM_DATA.sysname.find("Linux") < 0 ):
                print("-- GC Memory Alloc:", gc.mem_alloc(), "GC Memory Free:", gc.mem_free(), "GC Collected:", gc.collect());
            print ("Model: {:18}   HoldOver: {:11}   Service: {}".format( symModelArray["MODELTEXT"], holdState,symModelArray["SRVC"]) );
            print ("S/N: {:20}   Crystal: {:12}   In Voltage: {}".format( symModelArray["S/N"], symModelArray["CRYSTAL"], symStatusArray["DHTRVOLT"]) );
            print ("Rb Lock: {:16}   Current Temp: {:7.4}   Lamp Voltage: {:}".format( rblock, symStatusArray["DCURTEMP"], symStatusArray["DMP17"]) ) ;
            print ("PPS Delta: {:8} ({:3})   Adjusted: {:5} ({:3})   Counter: {}".format(symStatusArray["1PPSDELTA"], pps_rollover_correction(symStatusArray["1PPSDELTA"]), pps_rollover_correction(symStatusArray["1PPSDELTA"]) - symStatusArray["PPSOFFSET"], symStatusArray["PPSOFFSET"], symPPScounter));
            print ("R Slope: {:16}   P.Hours: {:12}   P.Ticks: {:}".format(slopeR, symStatusArray["PWRHRS"], symStatusArray["PWRTICKS"]) );
            print ("C Slope: {:16}   DDS Adjust: {:9}   Averages: {:}".format(slopeC, ddsAdjValue, dSums/DDS_DRIFT_WINDOW) );
            print("Sum Array: ", symSumsArray);
        else:
          time.sleep(0.01);  
        

    # Flush before quitting.
    get_serial_data(RUBIDIUM, 0);


# Start it up!
main();
