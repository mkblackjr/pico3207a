"""
Picoscope3207a Class.

This file contains the framework for the Picoscope3207a class. It makes calls
to the Picotech 3000a API in order to communicate with the Picoscope. The class
includes methods to collect data and save it in a csv form. Running this script
will instantiate the class, open the device, collect data in 100us blocks for 
5 seconds, save it, and then close.

Device parent class contains the following class variables:
    _name:              String  - name of device in ALL_CAPS (DEVICE)
    _opening:           Boolean - whether an attempt to open device was made
    _open:              Boolean - whether device was successfully opened
    _running:           Boolean - whether device was successfully started
    _allow_save:        Boolean - whether data to be collected will be saved
    _has_error:         Boolean - whether error has occurred in open, start,
                                  save, or update method
    _has_save_thread:   Boolean - whether device uses separate save thread
    _has_update_thread: Boolean - whether device uses separate update thread
    _lock:              multiprocessing.Lock - lock used start/stop methods 

Device parent class contains the following class properties:
    ready(): Boolean - maps to self._open

Device parent class contains the following public class methods:
    open():             Opens the device
    close():            Closes out the device
    start():            Starts updating and saving methods
    stop():             Stops updating and saving methods
    update():           Runs _update_device method on a background thread
    save():             Runs _save_device method on a background thread
    create_directory(): Creates new directories for saving device data
"""

__author__     = "Mitchell Black"
__copyright__  = "Copyright 2018, Michigan Aerospace Corporation"
__credits__    = ["Mitchell Black", "Picotech Support", "Picotech Github"]
__version__    = "1.0"
__maintainer__ = "Mitchell Black"
__email__      = "mblack@michiganaerospace.com"
__status__     = "Beta"

import traceback
import sys
import numpy as np
import inspect
import time
import csv
import datetime

from ctypes import *
from multiprocessing import Lock
from queue import Queue
from threading import Thread
from device import Device
from clockwork import clockwork
from error_codes import ERROR_CODES

LOOP_FREQ = 1 # Hz
LOOP_TIME = 1 / LOOP_FREQ
MAX_EXT = 32767
TRIGGER_ON = 1
TRIGGER_OFF = 0
CHANNEL_RANGE = [\
                {"rangeV": 20E-3,  "apivalue": 1,  "rangeStr": "20 mV"},
                {"rangeV": 50E-3,  "apivalue": 2,  "rangeStr": "50 mV"},
                {"rangeV": 100E-3, "apivalue": 3,  "rangeStr": "100 mV"},
                {"rangeV": 200E-3, "apivalue": 4,  "rangeStr": "200 mV"},
                {"rangeV": 500E-3, "apivalue": 5,  "rangeStr": "500 mV"},
                {"rangeV": 1.0,    "apivalue": 6,  "rangeStr": "1 V"},
                {"rangeV": 2.0,    "apivalue": 7,  "rangeStr": "2 V"},
                {"rangeV": 5.0,    "apivalue": 8,  "rangeStr": "5 V"},
                {"rangeV": 10.0,   "apivalue": 9,  "rangeStr": "10 V"},
                {"rangeV": 20.0,   "apivalue": 10, "rangeStr": "20 V"}]

class Picoscope3207a(Device):
    """ Picoscope3207a inherits from the Device class """

    def __init__(self):
        """ Initializes the class properties used throughout the Picoscope3207a
        class. Uses two locks, one for the run method and one for calls to the 
        API. Also instantiates two queue.Queue's, one to send data to the 
        process thread and one to transfer post-processed data to the save 
        thread. 

        Does not accept any arguments.

        Does not return any values.
        """
        super().__init__()
        self._name = "PICOSCOPE3207A"
        self._lib = None
        self._handle = None
        self._run_lock = Lock()
        self._driver_lock = Lock()

        self._collecting = False
        self._sampling_time = 1E-6
        self._sampling_duration = 100E-6
        self._samples = int(self._sampling_duration / self._sampling_time)
        self._idx = 0
        
        self._A_data = np.ones(100)*2
        self._B_data = np.ones(100)*-2
        self._t = np.linspace(0,100E-6,100)

        self._process_queue = Queue()
        self._save_queue = Queue()

    def _open_device(self):
        """ _open_device(): called by the parent Device class during the open()
        method. Loads the API functions and establishes communication to the 
        picoscope via the OpenUnit API function. Also switches the power source
        to USB Power if necessary.

        Does not accept any arguments.

        Returns True if successful.
        """
        self._lib = windll.LoadLibrary("C:\\Program Files\\Pico Technology\\SDK\\lib\\ps3000a.dll")
        c_handle = c_int16()
        with self._driver_lock:
            m = self._lib.ps3000aOpenUnit(byref(c_handle),None)
            if m == 286:
                m = self._lib.ps3000aChangePowerSource(c_handle,
                    c_int32(m))
        check_result(m)
        self._handle = c_handle

        return True

    def _close_device(self):
        """ _close_device(): called bu the parent Device class during the close()
        method. Disconnects from the picoscope via a call to the CloseUnit API
        function.

        Does not accept any arguments.

        Does not return any values.
        """
        with self._driver_lock:
            m = self._lib.ps3000aCloseUnit(self._handle)
        check_result(m)

    def _start_device(self):
        """ _start_device(): called by the parent Device class during the 
        start() method. Establishes the data and data_buffer class variables.
        Sets up the input channels, sets the Trigger, and sets the memory
        locations. Sets the Arbitrary Waveform Generator to output a square 
        wave of 40 kHz.

        Also responsible for starting the save, process, and collect threads in
        coordination with their respective queues.

        Does not accept any arguments.

        Returns True if successful.
        """
        self._data = [np.empty(self._samples,dtype=np.int16) for i in range(2)]
        self._data_buffer = [x.ctypes for x in self._data]
        self._timebase = self.get_timebase(self._sampling_time)
        self.v_range = CHANNEL_RANGE[7]["apivalue"] # 5V range
        with self._driver_lock:
            for i in range(2):  # two active channels
                m = self._lib.ps3000aSetChannel(self._handle,
                    c_int32(i), # channel
                    c_int16(1), # enabled
                    c_int32(1), # DC coupling
                    c_int32(self.v_range), 
                    c_float(0)) # 0V offset
                check_result(m)

                m = self._lib.ps3000aSetDataBuffer(self._handle,
                    c_int32(i),  # channel
                    self._data_buffer[i],
                    c_int32(self._samples),
                    c_uint32(0), # segment index
                    c_int32(0))  # ratio mode
                check_result(m)

            threshold_v = 0.1
            threshold_adc = int(threshold_v * MAX_EXT / self.v_range)
            m = self._lib.ps3000aSetSimpleTrigger(self._handle,
                c_int16(1),    # enabled
                c_int32(4),    # EXT trigger
                c_int16(threshold_adc),
                c_int32(2),    # direction = rising
                c_uint32(0),   # no delay
                c_int16(2000)) # autotrigger after 1 second if no trigger occurs
            check_result(m)

            # Send AWG Info to Picoscope
            output_freq = 40E3 # 40 kHz
            offset_voltage = 0
            pk2pk = 4
            wave_type = {'sine':0,'square':1,'triangle':2,'DC':3,
                         'rising sawtooth':4,'falling sawtooth':5,'sin(x)/x':6,
                         'Gaussian':7,'half-sine':8}
            trigger_type = 2 # siggen gate high
            trigger_source = 4 # software trigger
            m = self._lib.ps3000aSetSigGenBuiltIn(self._handle,
                c_int32(int(offset_voltage*1E6)), # offset voltage
                c_uint32(int(pk2pk*1E6)),# peak to peak voltage
                c_int32(wave_type['square']), # wave type
                c_float(output_freq), # start frequency
                c_float(output_freq), # stop frequency
                c_float(0), # increment
                c_float(0), # dwell count
                c_int32(0), # sweep type
                c_int32(0), # operation
                c_uint32(4), # shots
                c_uint32(0), # sweeps
                c_int32(trigger_type), 
                c_int32(trigger_source),
                c_int16(0)) # extIn threshold
            check_result(m)

        self._save_thread = Thread(target=self.save,args=(self._save_queue,))
        self._save_thread.daemon = True
        self._save_thread.start()

        self._process_thread = Thread(target=self.process,args=(self._process_queue,self._save_queue))
        self._process_thread.daemon = True
        self._process_thread.start()

        self._collect_thread = Thread(target=self.run_loop,args=(self._process_queue,))
        self._collect_thread.daemon = True
        self._collect_thread.start()

        return True

    def _stop_device(self):
        with self._driver_lock:
            m = self._lib.ps3000aStop(self._handle)
        check_result(m)

    def toggle_run(self):
        self._collecting = not self._collecting

    def run_loop(self,queue):
        while True:
            with self._run_lock:
                self.run(queue)
            time.sleep(0.001) # allow lock to be freed

    def run_once(self):
        with self._run_lock:
            self.run(self._process_queue,True)

    @clockwork(LOOP_TIME)
    def run(self,queue,override=False):
        if self._collecting or override:
            # if self._idx == 0:
            #     self._start_time = time.time()
            time_indisposed_ms = c_int32()
            ready = c_int16(0)
            with self._driver_lock:
                # Start Run
                m = self._lib.ps3000aRunBlock(self._handle,
                    c_int32(0), # pretrigger samples
                    c_int32(self._samples), # postrigger samples
                    c_uint32(self._timebase),
                    c_int16(0), # overflow - not used
                    byref(time_indisposed_ms), # time spent collecting data
                    c_uint32(0), # segment index
                    c_void_p(),
                    c_void_p())
                check_result(m)

                # Trigger AWG
                m = self._lib.ps3000aSigGenSoftwareControl(self._handle,c_int16(TRIGGER_ON))
                check_result(m)

                # Wait for picoscope
                while ready.value == 0:
                    m = self._lib.ps3000aIsReady(self._handle,byref(ready))
                    check_result(m)

                # Get Data
                n_samples = c_uint32(); n_samples.value = self._samples
                overflow = c_int16()
                for i in range(2):
                    start = i*self._samples
                    m = self._lib.ps3000aGetValues(self._handle,
                        c_uint32(start), # start index
                        byref(n_samples),
                        c_uint32(1),     # downsample ratio
                        c_int32(0),      # downsample ratio mode
                        c_uint32(0),     # segment index
                        byref(overflow)) # flags if channel has gone over voltage
                    check_result(m)

                # Get Trigger Offset
                times = c_int64()
                time_units = c_int32()
                m = self._lib.ps3000aGetTriggerTimeOffset64(self._handle,
                    byref(times),      # offset time
                    byref(time_units), # offset time unit
                    c_uint32(0))       # segment index
                check_result(m)

                # Re-arm AWG Trigger
                m = self._lib.ps3000aSigGenSoftwareControl(self._handle,c_int32(TRIGGER_OFF))
                check_result(m)

            offset_time = times.value * 10**(-15+3*time_units.value)

            time_indisposed_ms = time_indisposed_ms.value

            # Questionable Tactic
            if time_indisposed_ms > 0:
                time_data = np.linspace(0,time_indisposed_ms,self._samples) + offset_time
            else:
                time_data = np.linspace(0,self._sampling_duration,self._samples) + offset_time

            data = np.array(self._data) * self.v_range / MAX_EXT
            self._A_data = np.array(data[0])
            self._B_data = np.array(data[1])
            self._t = time_data

            # Place data into queue
            queue.put((time_data,data))
            # self._idx += 1

    def process(self,get_queue,put_queue):
        # idx = 0

        while True:
            try:
                t,v = get_queue.get()

                # do something to process data
                # if idx != 0:
                #     t += idx*self._sampling_duration

                put_queue.put((t,v))
            except:
                traceback.print_exc(file=sys.stdout)
            # idx += 1


    def save(self,queue):
        filename = datetime.datetime.utcnow().strftime('%Y%m%d_%H%M%S') + ".csv"
        with open(filename,'w',newline='') as csvfile:
            writer = csv.writer(csvfile,delimiter=',')
            writer.writerow(["Time (sec)","Channel A (V)","Channel B (V)"])

        while True:
            try:
                times,voltages = queue.get()

                # save in csv
                with open(filename,'a',newline='') as csvfile:
                    writer = csv.writer(csvfile,delimiter=',')
                    for t,v1,v2 in zip(times,voltages[0],voltages[1]):
                        writer.writerow([str(t),str(v1),str(v2)])
                
            except:
                traceback.print_exc(file=sys.stdout)


    def get_timebase(self,dt):
        from math import log
        if dt > 4E-9:
            n = int(dt*125E6 + 2)
        else:
            dt *= 1E9
            n = round(log(dt,2))
        print((n-2)/125000000)
        return n

    @property
    def t(self):
        return self._t

    @property
    def channel_data(self):
        return self._A_data,self._B_data

    @property
    def data1(self):
        return 1

    @property
    def data2(self):
        return 2

    @property
    def data3(self):
        return 3



###############################################################################
############################## Picoscope Channel ##############################
###############################################################################

class Channel():
    """ Picoscope Channel """
    CHANNEL_RANGE = [\
        {"rangeV": 20E-3,  "apivalue": 1,  "rangeStr": "20 mV"},
        {"rangeV": 50E-3,  "apivalue": 2,  "rangeStr": "50 mV"},
        {"rangeV": 100E-3, "apivalue": 3,  "rangeStr": "100 mV"},
        {"rangeV": 200E-3, "apivalue": 4,  "rangeStr": "200 mV"},
        {"rangeV": 500E-3, "apivalue": 5,  "rangeStr": "500 mV"},
        {"rangeV": 1.0,    "apivalue": 6,  "rangeStr": "1 V"},
        {"rangeV": 2.0,    "apivalue": 7,  "rangeStr": "2 V"},
        {"rangeV": 5.0,    "apivalue": 8,  "rangeStr": "5 V"},
        {"rangeV": 10.0,   "apivalue": 9,  "rangeStr": "10 V"},
        {"rangeV": 20.0,   "apivalue": 10, "rangeStr": "20 V"}]

    KEY = {'A':0,'B':1}

    max_samples = 0

    def __init__(self,handle,identity,dt,enabled=False):
        self._lib = LIB
        self._handle = handle

        self.id = identity
        self.enabled = enabled
        self.v_range = None
        self.v_offset = None
        self.coupling = 1 # DC, AC = 0
        self.segment = 0
        self.trigger = False
        self.timestep = dt

        # Initialize Channel to Off
        m = self._lib.ps2000aSetChannel(self._handle,
            c_int32(self.id),
            c_int16(0),
            c_int32(self.coupling),
            c_int32(5),
            c_float(0))
        check_result(m)

        #   # Get Channel Information
        # ranges = c_int32*4
        # length = c_int32(4)
        # i = 0
        # for key,chan in self._channels.keys():
        #     m = self._lib.ps2000aGetChannelInformation(self._handle,
        #         c_int32(0), # PS2000A_CHANNEL_INFO
        #         c_int32(0), # probe: not used, must be set to 0
        #         byref(ranges[i]),
        #         byref(length),
        #         c_int32(chan['id']))
        #     i += 1

    def enabled(self):
        self.enabled = True

    def disable(self):
        self.enabled = False

    def set(self,vr,vo):
        self.v_range = vr
        self.v_offset = vo        

        for v in self.CHANNEL_RANGE:
            if v["rangeV"] == vr:
                v_api = v["apivalue"]
                break

        if v_api is None:
            print("Channel {}: Voltage Range not an Option!".format(self.id))
        else: 
            self._v_api = v_api

        m = self._lib.ps2000aSetChannel(self._handle,
            c_int32(self.id),
            c_int16(self.enabled),
            c_int32(self.coupling),
            c_int32(self._v_api),
            c_float(vo))
        check_result(m)

        if self.enabled:
            print("Channel {} Enabled!".format(self.id))

    def set_trigger(self,threshold_v,direction,delay=0,auto=2000):
        threshold_adc = int(threshold_v/self.v_range * MAX_16_BIT)

        if not isinstance(direction,int):
            if direction is "Above":
                direction = 0
            elif direction is "Below":
                direction = 1
            elif direction is "Rising":
                direction = 2
            elif direction is "Falling":
                direction = 3
            elif direction is "RiseOrFall":
                direction = 4

        m = self._lib.ps2000aSetSimpleTrigger(self._handle,
            c_int16(self.enabled),
            c_int32(self.id),
            c_int16(threshold_adc),
            c_int32(direction),
            c_uint32(delay),
            c_int16(auto))
        check_result(m)

###############################################################################
######################## Arbitrary Waveform Generator #########################
###############################################################################

class AWG():
    """ Picoscope Arbitrary Waveform Generator """
    DDS_Freq = 20E6
    AWGPhaseAccumulatorSize = 2**32
    AWGBufferSize = 32768

    def __init__(self,handle,duration,dt):
        self._lib = LIB
        self._handle = handle

        self.waveform = None
        self.v_offset = 1
        self.pk_to_pk = 2
        self.delta_phase = None
        self.phase_increment = 0
        self.dwell_count = 0
        self.sweep_type = 0
        self.extra_operations = 0
        self.index_mode = 0
        self.shots = 1
        self.sweeps = 0
        self.trigger_source = None
        self.trigger_type = None
        self.ext_in = 0
        self.trigger = False

        self.timestep = dt
        self.duration = duration
        self.pulse_width = 100E-7
        self.pulse_location = 0.25

    def get_waveform(self,wtype='Pulse',width=100E-9,location=0.25):
        # Get AWG Information
        minWaveform = c_int16()
        maxWaveform = c_int16()
        minWaveformSize = c_uint32()
        maxWaveformSize = c_uint32()
        m = self._lib.ps2000aSigGenArbitraryMinMaxValues(self._handle,
            byref(minWaveform),
            byref(maxWaveform),
            byref(minWaveformSize),
            byref(maxWaveformSize))
        check_result(m)

        if wtype is not 'Pulse':
            print("Waveform Type Not Yet Supported")
            return 0
        else:
            duration = self.duration
            w_len = int(min(maxWaveformSize.value,duration/self.timestep))
            idx1 = int(w_len*(location - width/(2*duration)))
            idx2 = int(w_len*(location + width/(2*duration))) - 1
            waveform = np.array([MIN_16_BIT if (i < idx1 or i >= idx2) else MAX_16_BIT for i in range(w_len)],dtype=c_int16)

        return waveform,w_len

    def set(self,trgsrc,trgtype):
        self.waveform,self.length = self.get_waveform('Pulse',self.pulse_width,self.pulse_location)

        output_freq = 1/self.duration
        self.delta_phase = int((output_freq*self.AWGPhaseAccumulatorSize*self.length) / \
            (self.DDS_Freq * self.AWGBufferSize)) # 1 waveform per shot
        waveformPtr = self.waveform.ctypes

        if not isinstance(trgsrc,int):
            if trgsrc is "None":
                self.trigger_source = 0
            elif trgsrc is "ScopeTrig":
                self.trigger_source = 1
            elif trgsrc is "AuxIn":
                self.trigger_source = 2
            elif trgsrc is "ExtIn":
                self.trigger_source = 3
            elif trgsrc is "SoftTrig":
                self.trigger_source = 4
            elif trgsrc is "TriggerRaw":
                self.trigger_source = 5
        else:
            self.trigger_source = trgsrc

        if not isinstance(trgtype,int):
            if trgtype is "Rising":
                self.trigger_type = 0
            elif trgtype is "Falling":
                self.trigger_type = 1
            elif trgtype is "GateHigh":
                self.trigger_type = 2
            elif trgtype is "GateLow":
                self.trigger_type = 3
        else:
            self.trigger_type = trgtype

        # Send AWG Info to Picoscope
        m = self._lib.ps2000aSetSigGenArbitrary(self._handle,
            c_int32(int(self.v_offset*1E6)), 
            c_uint32(int(self.pk_to_pk*1E6)),
            c_uint32(self.delta_phase), # start delta phase
            c_uint32(self.delta_phase), # stop delta phase
            c_uint32(self.phase_increment), # delta phase increment
            c_uint32(self.dwell_count), # dwell count
            waveformPtr, # arbitrary waveform
            c_int32(self.length), # arbitrary waveform size
            c_int32(self.sweep_type), # sweep type for delta phase
            c_int32(self.extra_operations), # extra operations
            c_int32(self.index_mode), 
            c_uint32(self.shots), 
            c_uint32(self.sweeps),
            c_int32(self.trigger_type),
            c_int32(self.trigger_source),
            c_int16(self.ext_in)) # extIn threshold
        check_result(m)

###############################################################################
############################## Helper Functions ###############################
###############################################################################

def check_result(ec):
    """Check result of function calls, raise exception if not 0."""
    # NOTE: This will break some oscilloscopes that are powered by USB.
    # Some of the newer scopes, can actually be powered by USB and will
    # return a useful value. That should be given back to the user.
    # I guess we can deal with these edge cases in the functions themselves
    if ec == 0:
        return

    else:
        ecName = error_num_to_name(ec)
        ecDesc = error_num_to_desc(ec)
        raise IOError('Error calling %s: %s (%s)' % (
            str(inspect.stack()[1][3]), ecName, ecDesc))

def error_num_to_name(num):
    """Return the name of the error as a string."""
    for t in ERROR_CODES:
        if t[0] == num:
            return t[1]

def error_num_to_desc(num):
    """Return the description of the error as a string."""
    for t in ERROR_CODES:
        if t[0] == num:
            try:
                return t[2]
            except IndexError:
                return ""

###############################################################################
#################################### Main #####################################
###############################################################################

if __name__ == "__main__":
    picoscope = Picoscope3207a()
    picoscope.open()
    picoscope.start()

    time.sleep(5)

    picoscope.close()