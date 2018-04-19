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
    _data_queue:        queue.Queue - Queue for data
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
    restart():          Calls close() and open() with a delay in between
    toggle_save():      Toggles the boolean value _allow_save
    check_error():      Sets  _open, _running, _allow_save to False if error
    save():             Runs _save_device method on a background thread
    update():           Runs _update_device method on a background thread
"""

__author__     = "Mitchell Black"
__copyright__  = "Copyright 2018, Michigan Aerospace Corporation"
__credits__    = ["Picotech Support", "Picotech Github"]
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
from math import log

from device import Device
from clockwork import clockwork
from error_codes import ERROR_CODES

# Module-level variables
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
        self.distance = None

        self._process_queue = Queue()
        self._save_queue = Queue()

    def _open_device(self):
        """ Called by the parent Device class during the open() method. Loads 
        the API functions and establishes communication to the picoscope via 
        the OpenUnit API function. Also switches the power source to USB Power 
        if necessary.

        Does not accept any arguments.

        Returns True if successful.
        """
        self._lib = windll.LoadLibrary("lib\\ps3000a.dll")
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
        """ Called by the parent Device class during the close() method. 
        Disconnects from the picoscope via a call to the CloseUnit API 
        function.

        Does not accept any arguments.

        Does not return any values.
        """
        with self._driver_lock:
            m = self._lib.ps3000aCloseUnit(self._handle)
        check_result(m)

    def _start_device(self):
        """ Called by the parent Device class during the start() method. 
        Establishes the data and data_buffer class variables. Sets up the 
        input channels, sets the Trigger, and sets the memory locations. Sets
        the Arbitrary Waveform Generator to output a square wave of 40 kHz.

        Also responsible for starting the save, process, and collect threads in
        coordination with their respective queues.

        Does not accept any arguments.

        Returns True if successful.
        """
        self._data = [np.empty(self._samples,dtype=np.int16) for i in range(2)]
        self._data_buffer = [x.ctypes for x in self._data]
        self._timebase = self.get_timebase(self._sampling_time)
        self.v_rangeAPI = CHANNEL_RANGE[7]["apivalue"] # 5V range
        self.v_range = CHANNEL_RANGE[7]["rangeV"]
        with self._driver_lock:
            for i in range(2):  # two active channels
                m = self._lib.ps3000aSetChannel(self._handle,
                    c_int32(i), # channel
                    c_int16(1), # enabled
                    c_int32(1), # DC coupling
                    c_int32(self.v_rangeAPI), 
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
        """ Called by the parent Device class during the stop() method. Stops 
        collection mode via call to Stop API function.

        Does not accept any arguments.

        Does not return any values.
        """
        with self._driver_lock:
            m = self._lib.ps3000aStop(self._handle)
        check_result(m)

    def run_loop(self,queue):
        """ Target of the _collect_thread. Makes calls to the run() method to 
        acquire data.

        queue: queue.Queue() - self._process_queue to which data is added.

        Does not return any values.
        """
        while True:
            if self._running:
                with self._run_lock:
                    self.run(queue)
            time.sleep(0.001) # allow lock to be freed

    def run_once(self):
        """ Makes one call to the run() method.

        Does not accept any arguments.

        Does not return any values.
        """
        with self._run_lock:
            self.run(self._process_queue,True) # True: override flag for saving

    @clockwork(LOOP_TIME) # forces method below to execute in LOOP_TIME seconds
    def run(self,queue,override=False):
        """ Called to acquire data in Block mode. The following algorithm is 
        implemented: RunBlock -> SoftwareTriggerOn -> IsReady? -> GetValues ->
        GetTriggerTimeOffsets -> SoftwareTriggerOff -> add data to 
        _process_queue.

        queue: queue.Queue() - self._process_queue to which data is added for
               processing
        override: flag for save() method - save if run_once() method is called

        Does not return any values.
        """
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

        time_indisposed_s = time_indisposed_ms.value/1000
        time_indisposed_s = 0 # To Do: Determine what to do with this

        # Questionable Tactic
        if time_indisposed_s > 0:
            time_data = np.linspace(0,time_indisposed_s,self._samples) + offset_time
        else:
            time_data = np.linspace(0,self._sampling_duration,self._samples) + offset_time

        data = np.array(self._data) * self.v_range / MAX_EXT
        self._A_data = np.array(data[0])
        self._B_data = np.array(data[1])
        self._t = time_data
        print(max(data[0]))

        # Place data into queue
        queue.put((time_data,data,override))
        # self._idx += 1

    def process(self,get_queue,put_queue):
        """ Target of _process_thread. Processes the raw data collected from
        the picoscope.

        get_queue: queue.Queue() - self._process_queue from which data is taken
        put_queue: queue.Queue() - self._save_queue to which data is added 
                   for saving

        Does not return any values.
        """
        # idx = 0

        while True:
            try:
                t,v,override = get_queue.get()

                # do something to process data
                self.distance = v[0][1]


                put_queue.put((t,v,override))
            except:
                traceback.print_exc(file=sys.stdout)
            # idx += 1

    def save(self,queue):
        """ Target of _save_thread. Saves the processed data to a csv file.

        queue: queue.Queue() - self._save_queue from which data is taken

        Does not return any values.
        """
        filename = "data\\" + datetime.datetime.utcnow().strftime('%Y%m%d_%H%M%S') + ".csv"
        idx = 0

        while True:
            try:
                times,voltages,override = queue.get()

                # save in csv
                if self._collecting or override:
                    if idx == 0:
                        with open(filename,'w',newline='') as csvfile:
                            writer = csv.writer(csvfile,delimiter=',')
                            writer.writerow(["Time (sec)","Channel A (V)","Channel B (V)"])
                    with open(filename,'a',newline='') as csvfile:
                        writer = csv.writer(csvfile,delimiter=',')
                        for t,v1,v2 in zip(times,voltages[0],voltages[1]):
                            writer.writerow([str(t),str(v1),str(v2)])
                
            except:
                traceback.print_exc(file=sys.stdout)

            idx += 1

    def get_timebase(self,dt):
        """ Converts a delta_t (sampling time) into a timebase readable by the
        picoscope.

        dt: sampling time

        returns n: timebase
        """
        if dt > 4E-9:
            n = int(dt*125E6 + 2)
        else:
            dt *= 1E9
            n = round(log(dt,2))
        return n

    @property
    def t(self):
        return self._t

    @property
    def channel_data(self): 
        return self._A_data,self._B_data

    @property
    def data1(self):
        """ This is read by the pico_ui.py script """ 
        return round(max(self._A_data),3)

    @property
    def data2(self):
        """ This is read by the pico_ui.py script """
        return 2 # Insert relevant data field here

    @property
    def data3(self):
        """ This is read by the pico_ui.py script """
        return 3 # Insert relevant data field here

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