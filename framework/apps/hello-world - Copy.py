"""
    Hello World message sender as example application 
    for the ResFi framework.

    Copyright (C) 2016 Sven Zehl, Anatolij Zubow, Michael Doering, Adam Wolisz

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    {zehl, zubow, wolisz, doering}@tkn.tu-berlin.de
"""

__author__ = 'zehl, zubow, wolisz, doering'

import time
from common.resfi_api import AbstractResFiApp
import numpy as np
import math
import Queue
import os

class ResFiApp(AbstractResFiApp):

    def __init__(self, log, agent):
        AbstractResFiApp.__init__(self, log, "de.berlin.tu.tkn.hello-world", agent)
        # 802.11n parameters
        self.SPECTRAL_HT20_FFT_SIZE = 64
        self.SPECTRAL_HT20_NUM_BINS = 56
        self.SPECTRAL_HT20_40_NUM_BINS = 128
        self.SPECTRAL_HT20_40_FFT_SIZE = 128

        # ath9k spectral sample tlv type for numpy
        self.DTYPE_PSD_TLV = np.dtype([
                ("type", np.uint8),
                ("length", np.uint16),
                ]).newbyteorder('>')

        # ath9k spectral sample ht20 header type for numpy
        self.DTYPE_PSD_HDR_HT20 = np.dtype([
                ("type", np.uint8),
                ("length", np.uint16),
                ("max_exp", np.uint8),
                ("freq", np.uint16),
                ("rssi", np.int8),
                ("noise", np.int8),
                ("max_magnitude", np.uint16),
                ("max_index", np.uint8),
                ("bitmap_weight", np.uint8),
                ("tsf", np.uint64),
                ]).newbyteorder('>')
        # ath9k spectral sample ht40 header type for numpy
        self.DTYPE_PSD_HDR_HT20_40 = np.dtype([
                ("type", np.uint8),
                ("length", np.uint16),
                ("channel_type", np.uint8),
                ("freq", np.uint16),
                ("lower_rssi", np.int8),
                ("upper_rssi", np.int8),
                ("tsf", np.uint64),
                ("lower_noise", np.int8),
                ("upper_noise", np.int8),
                ("lower_max_magnitude", np.uint16),
                ("upper_max_magnitude", np.uint16),
                ("lower_max_index", np.uint8),
                ("upper_max_index", np.uint8),
                ("lower_bitmap_weight", np.uint8),
                ("upper_bitmap_weight", np.uint8),
                ("max_exp", np.uint8),
            ]).newbyteorder('>')


        # define sample type mappings
        self.SAMPLE_HDR_DTYPE = {
                1: self.DTYPE_PSD_HDR_HT20,
                2: self.DTYPE_PSD_HDR_HT20_40,
            }

        self.SAMPLE_NUM_BINS = {
                1: self.SPECTRAL_HT20_NUM_BINS,
                2: self.SPECTRAL_HT20_40_NUM_BINS,
            }

    """
                Decoder.py
    """
                
    def get_psd_vector_ht20(self,hdr, buf, debug=False):

        # check input
        if (len(buf) != (self.SPECTRAL_HT20_NUM_BINS)):
            warnings.warn('Invalid PSD buffer length detected.', RuntimeWarning, stacklevel=2)
            return np.array([], dtype=complex)
        elif buf.dtype != np.dtype(np.uint8):
            warnings.warn('Invalid PSD buffer data type detected.', RuntimeWarning, stacklevel=2)
            return np.array([], dtype=complex)
        else:
            buf = buf.tolist()
            max_exp = int(hdr['max_exp'])
            noise = int(hdr['noise'])
            rssi = int(hdr['rssi'])
            psd_vector = np.full((self.SPECTRAL_HT20_NUM_BINS), (np.nan), dtype=np.float64)

        # calculate unscaled (by max_exp) sum power over all FFT bins
        try:
            sum_pwr = 10*math.log10(sum([(x << max_exp)**2 for x in buf]))
        except :
            for sc_cnt, sc_dat in enumerate(buf):
                psd_vector[sc_cnt] = 0

        else: 
            # iterate over all FFT bins (subcarriers)
            for sc_cnt, sc_dat in enumerate(buf):
                if ( sc_dat == 0 ):
                    sc_dat = 1
                if debug:
                    print("Subcarrier Id %d -> noise=%d, rssi=%d, max_exp=%d, sc_dat=%d, sum_power=%d" % (sc_cnt, noise, rssi, max_exp, sc_dat, sum_pwr))
                sc_pwr = noise + rssi + 10*math.log10((sc_dat << max_exp)**2) - sum_pwr
                psd_vector[sc_cnt] = sc_pwr
        finally:
            return psd_vector


    def get_psd_vector_ht20_40(self,hdr, buf):
        pass


    def get_psd_vector(self,hdr, buf):

        GET_PSD_VECTOR = {
            1 : self.get_psd_vector_ht20,
            2 : self.get_psd_vector_ht20_40,
        }

        psd_vector = GET_PSD_VECTOR[int(hdr['type'])](hdr, buf)
        return psd_vector

        """
        Helper.py

        """
        
    def file_write(self,fn, msg):
        f = open(fn, 'w')
        f.write(msg)
        f.close()
        return None


    def get_device_driver(self,iface):
        """ get driver name for this interface """
        dn = '/sys/class/net/' + iface + '/device/driver/module'
        if ( os.path.isdir(dn) ):
            driver = os.path.split(os.readlink(dn))[1]
            return driver
        return None


    def get_device_phyid(self,iface):
        ''' get phy id for this interface '''
        fn = '/sys/class/net/' + iface + '/phy80211/name'
        if ( os.path.isfile(fn) ):
            f = open(fn, 'r')
            phyid = f.read().strip()
            f.close()
            return phyid
        return None
    
    def get_debugfs_dir(self,iface):
        ''' get debugfs directory for this interface '''
        phy_id = self.get_device_phyid(iface)
        driver = self.get_device_driver(iface)
        debugfs_dir = '/sys/kernel/debug/ieee80211/' + phy_id + '/' + driver + '/'
        if ( os.path.isdir(debugfs_dir) ):
            return debugfs_dir
        else:
            raise ValueError('Could not find debugfs directory for interface %s.' % iface)
        return None


    def scan_dev_start(self,iface):
        ''' enable ath9k built-in spectral analysis on this device '''
        debugfs_dir = self.get_debugfs_dir(iface)
        ctrl_fn = debugfs_dir + 'spectral_scan_ctl'
        cmd = 'trigger'
        self.file_write(ctrl_fn, cmd)
        return None


    def scan_dev_stop(self,iface):
        ''' disable ath9k built-in spectral analysis on this device '''
        debugfs_dir = self.get_debugfs_dir(iface)
        ctrl_fn = debugfs_dir + 'spectral_scan_ctl'
        cmd = 'disable'
        self.file_write(ctrl_fn, cmd)
        return None


    def scan_dev_drain(self,iface):
        ''' drain ath9k built-in spectral sample queue on this device '''
        debugfs_dir = self.get_debugfs_dir(iface)
        scan_fn = debugfs_dir + 'spectral_scan0'
        fd = open(scan_fn, 'rb')
        fd.read()
        fd.close()
        return None
    
    def scan_dev_configure(self,iface='wlan0', mode='manual', count=8, period=255, fft_period=15, short_repeat=1):
        ''' configure ath9k built-in spectral analysis on this device '''
        debugfs_dir = self.get_debugfs_dir(iface)

        # spectral_scan_ctl (scan mode)
        ctrl_fn = debugfs_dir + 'spectral_scan_ctl'
        cmd = str(mode)
        self.file_write(ctrl_fn, cmd)

        # spectral_count
        ctrl_fn = debugfs_dir + 'spectral_count'
        cmd = str(count)
        self.file_write(ctrl_fn, cmd)

        # spectral_period
        ctrl_fn = debugfs_dir + 'spectral_period'
        cmd = str(period)
        self.file_write(ctrl_fn, cmd)

        # spectral_fft_period
        ctrl_fn = debugfs_dir + 'spectral_fft_period'
        cmd = str(fft_period)
        self.file_write(ctrl_fn, cmd)

        # spectral_short_repeat
        ctrl_fn = debugfs_dir + 'spectral_short_repeat'
        cmd = str(short_repeat)
        self.file_write(ctrl_fn, cmd)

        """
        Scanner.py

        """
        

    def scan(self,iface='wlan0', q=Queue.Queue(), debug=False):

        # process input params
        debugfs_dir = self.get_debugfs_dir(iface)
        scan_fn = debugfs_dir + 'spectral_scan0'

        # init return
        psd_pkt = None

        # start receiving/decoding PSD packets
        if debug:
            print("Start reading PSD data from scan device...")

        # open PSD device and read all contents
        fd = open(scan_fn, 'rb')
        buf = fd.read()
        fd.close()

        # read and decode PSD device buffer if we have data in it
        if buf:

            seek = 0
            while seek < len(buf):

                # read TLV
                tlv_buf = buf[seek:seek+3]
                tlv = np.frombuffer(tlv_buf, dtype=self.DTYPE_PSD_TLV)
                tlv_len = self.DTYPE_PSD_TLV.itemsize

                # read sample header
                dtype_psd_hdr = self.SAMPLE_HDR_DTYPE[int(tlv['type'])]
                hdr_len = dtype_psd_hdr.itemsize
                hdr_buf = buf[seek:seek+hdr_len]
                hdr = np.frombuffer(hdr_buf, dtype=dtype_psd_hdr)

                if debug:
                    print("Reading PSD header: %s" % hdr)

                # read and decode PSD
                psd_num_bins = self.SAMPLE_NUM_BINS[int(hdr['type'])]
                psd_buf = buf[seek+hdr_len:seek+hdr_len+psd_num_bins]
                psd = np.frombuffer(psd_buf, dtype=np.uint8, count=psd_num_bins)
                psd_vector = self.get_psd_vector(hdr, psd)

                if debug:
                    print("Reading PSD vector:")
                    np.set_printoptions(formatter={'float': '{: 0.1f}'.format}, linewidth=120)
                    print(psd_vector)

                # combine data into common structure
                dtype_psd_pkt = np.dtype([
                    ("header", dtype_psd_hdr),
                    ("psd_vector", np.float64, (psd_num_bins)),
                ])

                psd_pkt = np.array([
                    (hdr, psd_vector),
                ], dtype=dtype_psd_pkt)

                #yield psd_pkt
                q.put(psd_pkt)

                # update seek
                sample_len = tlv_len + int(hdr['length'])   # total sample length [Byte]
                seek += sample_len                          # start of next sample

        else:
            if debug:
                print("PSD device buffer empty...")

        # finish
        if debug:
            print("Finished reading PSD data from scan device...")


    def run(self):
        self.log.debug("%s: plugin::spectralScan started ... " % self.agent.getNodeID())

        # define scan params
        iface = 'wlan0'
        runt = 5
        ival = 0.05
        debug = False
        spectral_mode = 'background'    # 'background', 'manual'
        spectral_count = 8              # default=8
        spectral_period = 255           # default=255, max=255
        spectral_fft_period = 15        # default=15, max=15
        spectral_short_repeat = 1       # default=1

        # configure scan device
        self.scan_dev_configure(
            iface=iface,
            mode=spectral_mode,
            count=spectral_count,
            period=spectral_period,
            fft_period=spectral_fft_period,
            short_repeat=spectral_short_repeat
            )
        # set timer
        start_time = time.time()
        end_time = start_time + runt

        # create plotter object
        #plt = psd_plotter.Plotter()

        # create scanner queue
        psdq = Queue.Queue()

        # start scan
        print("Start scanning PSD for %d sec in %d sec intervals on interface %s..." % (runt, ival, iface))
        if (spectral_mode == 'background'):
            self.scan_dev_start(iface)

        
        ival_cnt = 0
        while time.time() < end_time:

            if (spectral_mode == 'manual'):
                self.scan_dev_start(iface)

            # start scanner in a separate thread???
            self.scan(iface, psdq, debug)

            # collect all samples from last scan interval
            qsize = psdq.qsize()
            print("ival_cnt: %d, qsize: %d" % (ival_cnt, qsize))
            ret = np.full((56, qsize), (np.nan), dtype=np.float64)

            #while not myQueue.empty():
                #psd_pkt = myQueue.get()
                #print("Receiving PSD header: %s" % psd['header'])

            for i in range(0, qsize, 1):
                psd_pkt = psdq.get()
                ret[:,i] = psd_pkt['psd_vector']

            # calculate statistics for last scan interval
            avg = 10*np.log10(np.mean(10**np.divide(ret, 10), axis=1))
            env = np.max(ret, axis=1)

            if debug:
                print("Average power spectrum of last %d samples (%d sec):" % (qsize, ival))
                np.set_printoptions(formatter={'float': '{: 0.1f}'.format}, linewidth=120)
                print(avg)

                print("Envelope power spectrum of last %d samples (%d sec):" % (qsize, ival))
                np.set_printoptions(formatter={'float': '{: 0.1f}'.format}, linewidth=120)
                print(env)

            # update plotter
            #plt.updateplot(avg, env)

            # sleep for ival seconds
            ival_cnt += 1
            time.sleep(ival)

            self.log.debug("%s: plugin::hello-world started ... " % self.agent.getNodeID())

                    # send message to ResFi neighbors using ResFi northbound API
            my_msg = {}
            my_msg['psd'] = {'qsize' : qsize, 'avg' : ival}
            
            self.log.info("%s: plugin::hello-world sending %s to all one hop neighbors ... " 
                % (self.agent.getNodeID(), str(my_msg)))
            self.sendToNeighbors(my_msg, 1)
            
            time.sleep(1)

            self.log.debug("%s: plugin::hello-world stopped ... " % self.agent.getNodeID())




            #avg_cum = np.mean(avg, axis = 0)
            avgsend = {}
            for i in range(0,len(avg),1):
                avgsend[i] = avg[i]
            print str(avgsend)
            self.sendToNeighbors(avgsend, 1)
        #stop scan
        self.scan_dev_stop(iface)
        


    """
    receive callback function
    """
    def rx_cb(self, json_data):
        self.log.info("%s :: recv() msg from %s at %d: %s" % (self.ns, json_data['originator'], 
            json_data['tx_time_mus'], json_data))

    """
    new Link Notification Callback
    """
    def newLink_cb(self, nodeID):
        self.log.info("%s ::newLink_cb() new AP neighbor detected notification (newLink: %s)" 
            % (self.ns, nodeID))

    """
    Link Lost Notification Callback
    """
    def linkFailure_cb(self, nodeID):
        self.log.info("%s :: linkFailure_cb() neighbor AP disconnected (lostLink: %s)" 
            % (self.ns, nodeID))
