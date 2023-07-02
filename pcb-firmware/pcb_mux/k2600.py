"""
FILE: k2600.py
AUTHOR: R Ellingham
DATE MODIFIED: Feb 2021
PROGRAM DESC: Driver for communicating withe the Keithley2634B SMU. The program
has hard coded (yuck==True) specific Lua commands specified in the datasheet
but as a class of K2600. For example to call the Lua function
'serial.baud' you can now use 'K2600_instance.serial.baud' in your code.

Another version of this driver is yet to be made where any function can be
assumed as an attribute of K2600. For example 'K2600.function' is not explicitly
declared within the code but will be converted to a string, verfied as a valid
Lua command and then sent to the SMU via a pyvisa write or query command.

Uses pyvisa to form connection.

TODO:
1 - Hard code listed functions (copy 'display.measure.func' as a guide for other
funcs)
2 - Make simple write/read/query functions
3 - Make a function that checks for an error every time an object function is
called
"""
import sys
import pyvisa
import numpy as np
import time

import logging
from typing import (
    IO,
    Optional,
    Any,
    Dict,
    Union,
    List,
    Tuple,
    Sequence,
    Iterable,
    Iterator,
)

# Setup logger
formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")

_ch = logging.StreamHandler()
_ch.setFormatter(formatter)

logger = logging.getLogger(__name__)
logger.addHandler(_ch)

def debug_enable(stream_output: Optional[IO] = sys.stderr, level: int = logging.DEBUG) -> None:
    """
    Displays verbose logger ouput in console
    """
    logger.setLevel(level)
    _ch.setStream(stream_output)
    _ch.setLevel(level)

class K2600:
    """
    Contains some useful Lua commands:(add more as required)
    """
    def __init__(
        self,
        visa_address: str,
        raise_keithley_errors: bool = False,
        **kwargs,
        ) -> None:
        self.visa_address = visa_address
        self._connection_kwargs = kwargs

        self.raise_keithley_errors = raise_keithley_errors

        self.rm = pyvisa.ResourceManager()

        self.connect(**kwargs)

        self.connection.write("reset()")
        self.connection.write("errorqueue.clear()")

        self.connection.timeout = 2000
        try:
            self.connection.read_raw()
        except:
            pass
        print("SMU: " + self.connection.query("*IDN?").rstrip())


    def __repr__(self) -> str:
        return f"<{self.__class__.__name__}({self.visa_address})"

    def connect(self, **kwargs) -> bool:
        """
        Connects to Keithley device
        @param kwargs: Keyword arguments for Visa connection (e.g. baud_rate/
        timeout)
        @return Whether connection has succeeded
        """
        try:
            self.connection = self.rm.open_resource(self.visa_address,**kwargs)
            self.connection.read_termination = "\n"
            self.connected = True
            logger.debug("Connected to Keithley at %s.", self.visa_address)
        except ValueError:
            self.connection = None
            self.connected = False
            raise
        except ConnectionError:
            logger.info(
                "Connection error. Please check that no other program is connected."
            )
            self.connection = None
            self.connected = False
        except AttributeError:
            logger.info("Invalid VISA address %s.", self.visa_address)
            self.connection = None
            self.connected = False
        except Exception:
            logger.info("Could not connect to Keithley at %s.", self.visa_address)
            self.connection = None
            self.connected = False

        return self.connected

    def disconnect(self):
        """
        Disconnects Keithley device if present
        @return Connection status
        """
        self.smua.source.output(self,0)
        self.smub.source.output(self,0)
        logger.info("Turned off both a and b channels.")
        # Print any SMU errors
        have_errors = round(float(self.connection.query("print(errorqueue.count)").rstrip()))
        if have_errors > 0:
            print("SMU Errors:")
            for error in range(have_errors):
                self.connection.write("code, message = errorqueue.next()")
                print(self.connection.query("print(code,message)"))
        else:
            print("No smu errors")

        try:
            close_state = self.connection.close
            logger.info("Disconnected successfully from Keithley at %s.", self.visa_address)
        except AttributeError:
            logger.info("Can't disconnect because %s is not connected you silly goose!", self.visa_address)
            self.connection = None
            self.connected = False
        return self.connected

    def write(self, msg):
        """
        Write a specific message to the SMU
        """
        self.connection.write(msg)

    def read(self):
        """
        Write a specific message to the SMU
        """
        msg = self.connection.read()
        return msg

    def query(self, msg):
        """
        Write a specific message to the SMU and return the response
        """
        ans = self.connection.query(msg)
        return ans


    ### Hard-coded Lua functions as classes and functions ###
    """
    Usage of Pythonated-Lua functions:
    I) For using anyfunction an additional obj parameter will be required (e.g.
    Lua cmd "smuX.measure.Y(readingBuffer)" turns into python call
    "smuX.measure.Y(K2600_inst,readingBuffer)")

    II) If trying to obtain a stored parameter (e.g. Lua cmd: "display.smuX.measure.func")
    you will need to use a function to access this parameter (e.g. Python call
    "value = K2600_inst.display.smuX.measure.func(K2600_inst)")

    III) If trying to alter stored values you will need to parse a desired value
    for the stored parameter (e.g. Python call "K2600_inst.display.smuX.measure.func(K2600_inst, value)")

    IV) For functions that are assigned to variables (e.g. "buf_var = smua.makebuffer(buf_size)"),
    in python just whack in "K2600_inst.smua.makebuffer(K2600_inst,buf_size,buf_var)"

    V) Unsure what the f$#k I will do with Lua pointers at this stage
    """
    class beeper:
        def enable(self, value=None):
            if value != None:
                self.connection.write("beeper.enable = %d" % value)
            else:
                return self.connection.query("beeper.enable")
        def beep(self, length=1, freq=1000):
            self.connection.write("beeper.beep(%.2f,%.2f)" % (length, freq))

    def delay(self, time):
        self.connection.write("delay('%d')" % time)

    class display:
        MEASURE_DCAMPS = 0
        MEASURE_DCVOLTS = 1
        MEASURE_OHMS = 2
        MEASURE_WATTS = 3
        SMUA = 0
        SMUB = 1
        SMUA_SMUB = 2
        USER = 3
        def clear(self):
            self.connection.write("display.clear()")
        def settext(self, text: str):
            self.connection.write("display.settext('%s')" % text)
            print("display.settext('%s')" % text)
        def screen(self, value=None):
            if value != None:
                self.connection.write("display.screen = %d" % value)
            else:
                self.connection.write("screen_val = display.screen")
                return float(self.connection.query("print(screen_val)"))
        class smua:
            class measure:
                def func(self, value=None):
                    if value != None:
                        self.connection.write("display.smua.measure.func = %d" % value)
                    else:
                        self.connection.write("meas_func = display.smua.measure.func")
                        return float(self.connection.query("print(meas_func)"))


        class smub:
            class measure:
                def func(self, value=None):
                    if value != None:
                        self.connection.write("display.smub.measure.func = %d" % value)
                    else:
                        self.connection.write("meas_func = display.smub.measure.func")
                        return float(self.connection.query("print(meas_func)"))


    # def makegetter(self):
    #     self.connection.write("makegetter()")
    #
    # def makegetter(self):
    #     self.connection.write("makegetter()")

    def print(self, value):
        return self.connection.query("print(value)")

    # def PulseIMeasureV(self):
    #     self.connection.write("PulseIMeasureV()")
    #
    # class os:
    #     def time(self):
    #         self.connection.write("os.time()")
    #
    class smua:
        FILTER_OFF = 0
        FILTER_ON = 1

        SENSE_LOCAL = 0
        SENSE_REMOTE = 1
        SENSE_CALA = 3

        OUTPUT_DCAMPS = 0
        OUTPUT_DCVOLTS = 1

        def makebuffer(self,buf_size,buf_var):
            self.connection.write("%s = smua.makebuffer(%d)" % (buf_var, buf_size))
        class measure:
            def v(self, buf_var=None):
                if buf_var != None:
                    self.connection.write("reading = smua.measure.v(%s)" % buf_var)
                    return self.connection.query("print(reading)")
                else:
                    self.connection.write("reading = smua.measure.v()")
                    return self.connection.query("print(reading)")
            def i(self, buf_var=None):
                if buf_var != None:
                    self.connection.write("reading = smua.measure.i(%s)" % buf_var)
                    return self.connection.query("print(reading)")
                else:
                    self.connection.write("reading = smua.measure.i()")
                    return self.connection.query("print(reading)")
            def r(self, buf_var=None):
                if buf_var != None:
                    self.connection.write("reading = smua.measure.r(%s)" % buf_var)
                    return self.connection.query("print(reading)")
                else:
                    self.connection.write("reading = smua.measure.r()")
                    return self.connection.query("print(reading)")
            def iv(self): # TODO:Look in Keithley 2600 ref manual
                    self.connection.write("reading = smua.measure.iv()")
                    return self.connection.query("print(reading)")
            def nplc(self,value):
                if value != None:
                    self.connection.write("smua.measure.nplc = %f" % value)
                else:
                    self.connection.write("nplc_var = smua.measure.nplc")
                    return self.connection.query("print(nplc_var)")

            class filter:
                def enable(self, value=None):
                    if value != None:
                        self.connection.write("smua.measure.filter.enable = %d" % value)
                    else:
                        self.connection.write("enable_var = smua.measure.filter.enable")
                        return self.connection.query("print(enable_var)")
                def type(self, value=None):
                    if value != None:
                        self.connection.write("smua.measure.filter.type = %d" % value)
                    else:
                        self.connection.write("type_var = smua.measure.filter.type")
                        return self.connection.query("print(type_var)")

        def reset(self):
            self.connection.write("smua.reset()")

        def savebuffer(self, smuX_buf):
            # smuX_buf must be a string in format "smuX.nvbufferY" described
            # below in Lua format:
            # smuX.savebuffer(smuX.nvbufferY)
            # X SMU channel (for example, smua.savebuffer(smua.nvbuffer1) applies to
            # SMU channel A)
            # Y SMU dedicated reading buffer (1 or 2)
            self.connection.write("smua.savebuffer()")

        def sense(self, value=None):
            if value != None:
                self.connection.write("smua.sense = %d" % value)
            else:
                self.connection.write("sense_var = smua.sense")
                return self.connection.query("print(sense_var)")
                
        class source:
            def levelv(self, value=None):
                if value != None:
                    self.connection.write("smua.source.levelv = %.5f" % value)
                else:
                    self.connection.write("levelv_var = smua.source.levelv")
                    return self.connection.query("print(levelv_var)")
            def leveli(self, value=None):
                if value != None:
                    self.connection.write("smua.source.leveli = %.5f" % value)
                else:
                    self.connection.write("leveli_var = smua.source.leveli")
                    return self.connection.query("print(leveli_var)")
            def limitv(self, value=None):
                if value != None:
                    self.connection.write("smua.source.limitv = %.5f" % value)
                else:
                    self.connection.write("limitv_var = smua.source.limitv")
                    return self.connection.query("print(limitv_var)")
            def limiti(self, value=None):
                if value != None:
                    self.connection.write("smua.source.limiti = %.5f" % value)
                else:
                    self.connection.write("limiti_var = smua.source.limiti")
                    return self.connection.query("print(limiti_var)")
            def func(self, value=None):
                if value != None:
                    self.connection.write("smua.source.func = %d" % value)
                else:
                    self.connection.write("func_var = smua.source.func")
                    return self.connection.query("print(func_var)")
            def output(self, value=None):
                if value != None:
                    self.connection.write("smua.source.output = %d" % value)
                else:
                    self.connection.write("output_var = smua.source.output")
                    return self.connection.query("print(output_var)")

    class smub:
        FILTER_OFF = 0
        FILTER_ON = 1

        SENSE_LOCAL = 0
        SENSE_REMOTE = 1
        SENSE_CALA = 3

        OUTPUT_DCAMPS = 0
        OUTPUT_DCVOLTS = 1

        def makebuffer(self,buf_size,buf_var):
            self.connection.write("%s = smub.makebuffer(%d)" % (buf_var, buf_size))
        class measure:
            def v(self, buf_var=None):
                if buf_var != None:
                    self.connection.write("reading = smub.measure.v(%s)" % buf_var)
                    return self.connection.query("print(reading)")
                else:
                    self.connection.write("reading = smub.measure.v()")
                    return self.connection.query("print(reading)")
            def i(self, buf_var=None):
                if buf_var != None:
                    self.connection.write("reading = smub.measure.i(%s)" % buf_var)
                    return self.connection.query("print(reading)")
                else:
                    self.connection.write("reading = smub.measure.i()")
                    return self.connection.query("print(reading)")
            def r(self, buf_var=None):
                if buf_var != None:
                    self.connection.write("reading = smub.measure.r(%s)" % buf_var)
                    return self.connection.query("print(reading)")
                else:
                    self.connection.write("reading = smub.measure.r()")
                    return self.connection.query("print(reading)")
            def iv(self): # TODO:Look in Keithley 2600 ref manual
                    self.connection.write("reading = smub.measure.iv()")
                    return self.connection.query("print(reading)")
            def nplc(self,value):
                if value != None:
                    self.connection.write("smub.measure.nplc = %f" % value)
                else:
                    self.connection.write("nplc_var = smub.measure.nplc")
                    return self.connection.query("print(nplc_var)")

            class filter:
                def enable(self, value=None):
                    if value != None:
                        self.connection.write("smub.measure.filter.enable = %d" % value)
                    else:
                        self.connection.write("enable_var = smub.measure.filter.enable")
                        return self.connection.query("print(enable_var)")
                def type(self, value=None):
                    if value != None:
                        self.connection.write("smub.measure.filter.type = %d" % value)
                    else:
                        self.connection.write("type_var = smub.measure.filter.type")
                        return self.connection.query("print(type_var)")

        def reset(self):
            self.connection.write("smub.reset()")

        def savebuffer(self, smuX_buf):
            # smuX_buf must be a string in format "smuX.nvbufferY" described
            # below in Lua format:
            # smuX.savebuffer(smuX.nvbufferY)
            # X SMU channel (for example, smub.savebuffer(smub.nvbuffer1) applies to
            # SMU channel A)
            # Y SMU dedicated reading buffer (1 or 2)
            self.connection.write("smub.savebuffer()")

        def sense(self, value=None):
            if value != None:
                self.connection.write("smub.sense = %d" % value)
            else:
                self.connection.write("sense_var = smub.sense")
                return self.connection.query("print(sense_var)")

        class source:
            def levelv(self, value=None):
                if value != None:
                    self.connection.write("smub.source.levelv = %.5f" % value)
                else:
                    self.connection.write("levelv_var = smub.source.levelv")
                    return self.connection.query("print(levelv_var)")
            def leveli(self, value=None):
                if value != None:
                    self.connection.write("smub.source.leveli = %.5f" % value)
                else:
                    self.connection.write("leveli_var = smub.source.leveli")
                    return self.connection.query("print(leveli_var)")
            def limitv(self, value=None):
                if value != None:
                    self.connection.write("smub.source.limitv = %.5f" % value)
                else:
                    self.connection.write("limitv_var = smub.source.limitv")
                    return self.connection.query("print(limitv_var)")
            def limiti(self, value=None):
                if value != None:
                    self.connection.write("smub.source.limiti = %.5f" % value)
                else:
                    self.connection.write("limiti_var = smub.source.limiti")
                    return self.connection.query("print(limiti_var)")
            def func(self, value=None):
                if value != None:
                    self.connection.write("smub.source.func = %d" % value)
                else:
                    self.connection.write("func_var = smub.source.func")
                    return self.connection.query("print(func_var)")
            def output(self, value=None):
                if value != None:
                    self.connection.write("smub.source.output = %d" % value)
                else:
                    self.connection.write("output_var = smub.source.output")
                    return self.connection.query("print(output_var)")
    
    class serial:
        def baud(self,value=None):
            if value != None:
                self.connection.write("serial.baud = %d" % value)
            else:
                self.connection.write("baud_var = serial.baud")
                return self.connection.query("print(baud_var)")

    class timer:
        def reset(self):
            self.connection.write("timer.reset()")
        class measure:
            def t(self):
                self.connection.write("t_var = timer.measure.t()")
                return self.connection.query("print(t_var)")




# debug_enable()

# k = K2600('TCPIP0::169.254.0.1::inst0::INSTR')
#
# k.display.smua.measure.func(k,k.display.MEASURE_OHMS)
# alrighty = k.display.smua.measure.func(k)
#
# k.disconnect()
