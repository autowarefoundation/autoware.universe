#!/usr/bin/env python
import serial
import struct
import sys
import getopt
import time
import datetime
import glob
import re
import pprint

from mtdef import MID, OutputMode, OutputSettings, MTException, Baudrates, \
    XDIGroup, getMIDName, DeviceState, DeprecatedMID, MTErrorMessage, \
    MTTimeoutException


################################################################
# MTDevice class
################################################################
class MTDevice(object):
    """XSens MT device communication object."""

    def __init__(self, port, baudrate=115200, timeout=0.002, autoconf=True,
                 config_mode=False, verbose=False, initial_wait=0.1):
        """Open device."""
        self.verbose = verbose
        # serial interface to the device
        try:
            self.device = serial.Serial(port, baudrate, timeout=timeout,
                                        writeTimeout=timeout)
        except IOError:
            # FIXME with pyserial3 we might need some specific flags
            self.device = serial.Serial(port, baudrate, timeout=timeout,
                                        writeTimeout=timeout, rtscts=True,
                                        dsrdtr=True)
        time.sleep(initial_wait)  # open returns before device is ready
        self.device.flushInput()
        self.device.flushOutput()
        # timeout for communication
        self.timeout = 100 * timeout
        # state of the device
        self.state = None
        if autoconf:
            self.auto_config_legacy()
        else:
            # mode parameter of the IMU
            self.mode = None
            # settings parameter of the IMU
            self.settings = None
            # length of the MTData message
            self.length = None
            # header of the MTData message
            self.header = None
        if config_mode:
            self.GoToConfig()

    ############################################################
    # Low-level communication
    ############################################################
    def write_msg(self, mid, data=b''):
        """Low-level message sending function."""
        length = len(data)
        if length > 254:
            lendat = b'\xFF' + struct.pack('!H', length)
        else:
            lendat = struct.pack('!B', length)
        packet = b'\xFA\xFF' + struct.pack('!B', mid) + lendat + data
        packet += struct.pack('!B', 0xFF & (-(sum(packet[1:]))))
        msg = packet
        start = time.time()
        while ((time.time() - start) < self.timeout) and self.device.read():
            pass
        try:
            self.device.write(msg)
        except serial.serialutil.SerialTimeoutException:
            raise MTTimeoutException("writing message")
        if self.verbose:
            print("MT: Write message id 0x%02X (%s) with %d data bytes: " \
                  "[%s]" % (mid, getMIDName(mid), length,
                            ' '.join("%02X" % ord(v) for v in data)))

    def waitfor(self, size=1):
        """Get a given amount of data."""
        buf = bytearray()
        for _ in range(100):
            buf.extend(self.device.read(size - len(buf)))
            if len(buf) == size:
                return buf
            if self.verbose:
                print("waiting for %d bytes, got %d so far: [%s]" % \
                      (size, len(buf), ' '.join('%02X' % v for v in buf)))
        raise MTTimeoutException("waiting for message")

    def read_data_msg(self, buf=bytearray()):
        """Low-level MTData receiving function.
        Take advantage of known message length.
        """
        start = time.time()
        if self.length <= 254:
            totlength = 5 + self.length
        else:
            totlength = 7 + self.length
        while (time.time() - start) < self.timeout:
            buf.extend(self.waitfor(totlength - len(buf)))
            preamble_ind = buf.find(self.header)
            if preamble_ind == -1:  # not found
                # discard unexploitable data
                if self.verbose:
                    sys.stderr.write("MT: discarding (no preamble).\n")
                del buf[:-3]
                continue
            elif preamble_ind:  # found but not at start
                # discard leading bytes
                if self.verbose:
                    sys.stderr.write("MT: discarding (before preamble).\n")
                del buf[:preamble_ind]
                # complete message for checksum
                buf.extend(self.waitfor(totlength - len(buf)))
            if 0xFF & sum(buf[1:]):
                if self.verbose:
                    sys.stderr.write("MT: invalid checksum; discarding data "
                                     "and waiting for next message.\n")
                del buf[:buf.find(self.header) - 2]
                continue
            data = str(buf[-self.length - 1:-1])
            del buf[:]
            return data
        else:
            raise MTException("could not find MTData message.")

    def read_msg(self):
        """Low-level message receiving function."""
        start = time.time()
        while (time.time() - start) < self.timeout:
            # first part of preamble
            if ord(self.waitfor()) != 0xFA:
                continue
            # second part of preamble
            if ord(self.waitfor()) != 0xFF:  # we assume no timeout anymore
                continue
            # read message id and length of message
            mid, length = struct.unpack('!BB', self.waitfor(2))
            if length == 255:  # extended length
                length, = struct.unpack('!H', self.waitfor(2))
            # read contents and checksum
            buf = self.waitfor(length + 1)
            checksum = buf[-1]
            data = struct.unpack('!%dB' % length, buf[:-1])
            # check message integrity
            if 0xFF & sum(data, 0xFF + mid + length + checksum):
                if self.verbose:
                    sys.stderr.write("invalid checksum; discarding data and "
                                     "waiting for next message.\n")
                continue
            if self.verbose:
                print("MT: Got message id 0x%02X (%s) with %d data bytes: " \
                      "[%s]" % (mid, getMIDName(mid), length,
                                ' '.join("%02X" % v for v in data)))
            if mid == MID.Error:
                raise MTErrorMessage(data[0])
            return (mid, buf[:-1])
        else:
            raise MTException("could not find message.")

    def write_ack(self, mid, data=b'', n_resend=30, n_read=25):
        """Send a message and read confirmation."""
        for _ in range(n_resend):
            self.write_msg(mid, data)
            for _ in range(n_read):
                mid_ack, data_ack = self.read_msg()
                if mid_ack == (mid + 1):
                    break
                elif self.verbose:
                    print("ack (0x%02X) expected, got 0x%02X instead" % \
                          (mid + 1, mid_ack))
            else:  # inner look not broken
                continue  # retry (send+wait)
            break  # still no luck
        else:
            n_retries = n_resend * n_read
            raise MTException("Ack (0x%02X) expected, MID 0x%02X received "
                              "instead (after %d retries)." % (mid + 1, mid_ack,
                                                               n_retries))
        return data_ack

    def _ensure_config_state(self):
        """Switch device to config state if necessary."""
        if self.state != DeviceState.Config:
            self.GoToConfig()

    def _ensure_measurement_state(self):
        """Switch device to measurement state if necessary."""
        if self.state != DeviceState.Measurement:
            self.GoToMeasurement()

    ############################################################
    # High-level functions
    ############################################################
    def Reset(self, go_to_config=False):
        """Reset MT device.

        If go_to_config then send WakeUpAck in order to leave the device in
        config mode.
        """
        self.write_ack(MID.Reset)
        if go_to_config:
            time.sleep(0.01)
            mid, _ = self.read_msg()
            if mid == MID.WakeUp:
                self.write_msg(MID.WakeUpAck)
                self.state = DeviceState.Config
        else:
            self.state = DeviceState.Measurement

    def GoToConfig(self):
        """Place MT device in configuration mode."""
        self.write_ack(MID.GoToConfig)
        self.state = DeviceState.Config

    def GoToMeasurement(self):
        """Place MT device in measurement mode."""
        self._ensure_config_state()
        self.write_ack(MID.GoToMeasurement)
        self.state = DeviceState.Measurement

    def GetDeviceID(self):
        """Get the device identifier."""
        self._ensure_config_state()
        data = self.write_ack(MID.ReqDID)
        deviceID, = struct.unpack('!I', data)
        return deviceID

    def GetProductCode(self):
        """Get the product code."""
        self._ensure_config_state()
        data = self.write_ack(MID.ReqProductCode)
        return str(data).strip()

    def GetHardwareVersion(self):
        """Get the hardware version."""
        self._ensure_config_state()
        data = self.write_ack(MID.ReqHardwareVersion)
        major, minor = struct.unpack('!BB', data)
        return (major, minor)

    def GetFirmwareRev(self):
        """Get the firmware version."""
        self._ensure_config_state()
        data = self.write_ack(MID.ReqFWRev)
        if len(data) == 3:
            major, minor, revision = struct.unpack('!BBB', data)
            return (major, minor, revision)
        else:
            # TODO check buildnr/svnrev not sure unsigned
            major, minor, rev, buildnr, svnrev = struct.unpack('!BBBII', data)
            return (major, minor, rev, buildnr, svnrev)

    def RunSelfTest(self):
        """Run the built-in self test."""
        self._ensure_config_state()
        data = self.write_ack(MID.RunSelfTest)
        bit_names = ['accX', 'accY', 'accZ', 'gyrX', 'gyrY', 'gyrZ',
                     'magX', 'magY', 'magZ']
        self_test_results = []
        for i, name in enumerate(bit_names):
            self_test_results.append((name, (data >> i) & 1))
        return self_test_results

    def GetBaudrate(self):
        """Get the current baudrate id of the device."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetBaudrate)
        return data[0]

    def SetBaudrate(self, brid):
        """Set the baudrate of the device using the baudrate id."""
        self._ensure_config_state()
        self.write_ack(MID.SetBaudrate, (brid,))

    def GetErrorMode(self):
        """Get the current error mode of the device."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetErrorMode)
        error_mode, = struct.unpack('!H', data)
        return error_mode

    def SetErrorMode(self, error_mode):
        """Set the error mode of the device.

        The error mode defines the way the device deals with errors (expect
        message errors):
            0x0000: ignore any errors except message handling errors,
            0x0001: in case of missing sampling instance: increase sample
                counter and do not send error message,
            0x0002: in case of missing sampling instance: increase sample
                counter and send error message,
            0x0003: in case of non-message handling error, an error message is
                sent and the device will go into Config State.
        """
        self._ensure_config_state()
        data = struct.pack('!H', error_mode)
        self.write_ack(MID.SetErrorMode, data)

    def GetOptionFlags(self):
        """Get the option flags (MTi-1 series)."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetOptionFlags)
        flags, = struct.unpack('!I', data)
        return flags

    def SetOptionFlags(self, set_flags, clear_flags):
        """Set the option flags (MTi-1 series)."""
        self._ensure_config_state()
        data = struct.pack('!II', set_flags, clear_flags)
        self.write_ack(MID.SetOptionFlags, data)

    def GetLocationID(self):
        """Get the location ID of the device."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetLocationID)
        location_id, = struct.unpack('!H', data)
        return location_id

    def SetLocationID(self, location_id):
        """Set the location ID of the device (arbitrary)."""
        self._ensure_config_state()
        data = struct.pack('!H', location_id)
        self.write_ack(MID.SetLocationID, data)

    def RestoreFactoryDefaults(self):
        """Restore MT device configuration to factory defaults (soft version).
        """
        self._ensure_config_state()
        self.write_ack(MID.RestoreFactoryDef)

    def GetTransmitDelay(self):
        """Get the transmission delay (only RS485)."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetTransmitDelay)
        transmit_delay, = struct.unpack('!H', data)
        return transmit_delay

    def SetTransmitDelay(self, transmit_delay):
        """Set the transmission delay (only RS485)."""
        self._ensure_config_state()
        data = struct.pack('!H', transmit_delay)
        self.write_ack(MID.SetTransmitDelay, data)

    def GetSyncSettings(self):
        """Get the synchronisation settings."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetSyncSettings)
        sync_settings = [struct.unpack('!BBBBHHHH', data[o:o + 12])
                         for o in range(0, len(data), 12)]
        return sync_settings

    def SetSyncSettings(self, sync_settings):
        """Set the synchronisation settings (mark IV)"""
        self._ensure_config_state()
        data = b''.join(struct.pack('!BBBBHHHH', *sync_setting)
                        for sync_setting in sync_settings)
        self.write_ack(MID.SetSyncSettings, data)

    def GetConfiguration(self):
        """Ask for the current configuration of the MT device."""
        self._ensure_config_state()
        config = self.write_ack(MID.ReqConfiguration)
        try:
            masterID, period, skipfactor, _, _, _, date, time, num, deviceID, \
            length, mode, settings = \
                struct.unpack('!IHHHHI8s8s32x32xHIHHI8x', config)
        except struct.error:
            raise MTException("could not parse configuration.")
        self.mode = mode
        self.settings = settings
        self.length = length
        if self.length <= 254:
            self.header = b'\xFA\xFF\x32' + struct.pack('!B', self.length)
        else:
            self.header = b'\xFA\xFF\x32\xFF' + struct.pack('!H', self.length)
        conf = {'output-mode': mode,
                'output-settings': settings,
                'length': length,
                'period': period,
                'skipfactor': skipfactor,
                'Master device ID': masterID,
                'date': date,
                'time': time,
                'number of devices': num,
                'device ID': deviceID}
        return conf

    def GetOutputConfiguration(self):
        """Get the output configuration of the device (mark IV)."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetOutputConfiguration)
        output_configuration = [struct.unpack('!HH', data[o:o + 4])
                                for o in range(0, len(data), 4)]
        return output_configuration

    def SetOutputConfiguration(self, output_configuration):
        """Set the output configuration of the device (mark IV)."""
        self._ensure_config_state()
        data = b''.join(struct.pack('!HH', *output)
                        for output in output_configuration)
        self.write_ack(MID.SetOutputConfiguration, data)

    def GetStringOutputType(self):
        """Get the NMEA data output configuration."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetStringOutputType)
        string_output_type, = struct.unpack('!H', data)
        return string_output_type

    def SetStringOutputType(self, string_output_type):
        """Set the configuration of the NMEA data output."""
        self._ensure_config_state()
        data = struct.pack('!H', string_output_type)
        self.write_ack(MID.SetStringOutputType, data)

    def GetPeriod(self):
        """Get the sampling period."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetPeriod)
        period, = struct.unpack('!H', data)
        return period

    def SetPeriod(self, period):
        """Set the sampling period."""
        self._ensure_config_state()
        data = struct.pack('!H', period)
        self.write_ack(MID.SetPeriod, data)

    def GetAlignmentRotation(self, parameter):
        """Get the object alignment.

        parameter indicates the desired alignment quaternion:
            0 for sensor alignment (RotSensor),
            1 for local alignment (RotLocal).
        """
        self._ensure_config_state()
        data = struct.pack('!B', parameter)
        data = self.write_ack(MID.SetAlignmentRotation, data)
        if len(data) == 16:  # fix for older firmwares
            q0, q1, q2, q3 = struct.unpack('!ffff', data)
            return parameter, (q0, q1, q2, q3)
        elif len(data) == 17:
            param, q0, q1, q2, q3 = struct.unpack('!Bffff', data)
            return param, (q0, q1, q2, q3)
        else:
            raise MTException('Could not parse ReqAlignmentRotationAck message:'
                              ' wrong size of message (%d instead of either 16 '
                              'or 17).' % len(data))

    def SetAlignmentRotation(self, parameter, quaternion):
        """Set the object alignment.

        parameter indicates the desired alignment quaternion:
            0 for sensor alignment (RotSensor),
            1 for local alignment (RotLocal).
        """
        self._ensure_config_state()
        data = struct.pack('!Bffff', parameter, *quaternion)
        self.write_ack(MID.SetAlignmentRotation, data)

    def GetOutputMode(self):
        """Get current output mode."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetOutputMode)
        self.mode, = struct.unpack('!H', data)
        return self.mode

    def SetOutputMode(self, mode):
        """Select which information to output."""
        self._ensure_config_state()
        data = struct.pack('!H', mode)
        self.write_ack(MID.SetOutputMode, data)
        self.mode = mode

    def GetExtOutputMode(self):
        """Get current extended output mode (for alternative UART)."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetExtOutputMode)
        ext_mode, = struct.unpack('!H', data)
        return ext_mode

    def SetExtOutputMode(self, ext_mode):
        """Set extended output mode (for alternative UART)."""
        self._ensure_config_state()
        data = struct.pack('!H', ext_mode)
        self.write_ack(MID.SetExtOutputMode, data)

    def GetOutputSettings(self):
        """Get current output mode."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetOutputSettings)
        self.settings, = struct.unpack('!I', data)
        return self.settings

    def SetOutputSettings(self, settings):
        """Select how to output the information."""
        self._ensure_config_state()
        data = struct.pack('!I', settings)
        self.write_ack(MID.SetOutputSettings, data)
        self.settings = settings

    def SetOutputSkipFactor(self, skipfactor):  # deprecated
        """Set the output skip factor."""
        self._ensure_config_state()
        data = struct.pack('!H', skipfactor)
        self.write_ack(DeprecatedMID.SetOutputSkipFactor, data)

    def ReqDataLength(self):  # deprecated
        """Get data length for mark III devices."""
        self._ensure_config_state()
        try:
            data = self.write_ack(DeprecatedMID.ReqDataLength)
        except MTErrorMessage as e:
            if e.code == 0x04:
                sys.stderr.write("ReqDataLength message is deprecated and not "
                                 "recognised by your device.")
                return
            raise e
        self.length, = struct.unpack('!H', data)
        if self.length <= 254:
            self.header = b'\xFA\xFF\x32' + struct.pack('!B', self.length)
        else:
            self.header = b'\xFA\xFF\x32\xFF' + struct.pack('!H', self.length)
        return self.length

    def GetLatLonAlt(self):
        """Get the stored position of the device.
        It is used internally for local magnetic declination and local gravity.
        """
        self._ensure_config_state()
        data = self.write_ack(MID.SetLatLonAlt)
        if len(data) == 24:
            lat, lon, alt = struct.unpack('!ddd', data)
        elif len(data) == 12:
            lat, lon, alt = struct.unpack('!fff', data)
        else:
            raise MTException('Could not parse ReqLatLonAltAck message: wrong'
                              'size of message.')
        return (lat, lon, alt)

    def SetLatLonAlt(self, lat, lon, alt):
        """Set the position of the device.
        It is used internally for local magnetic declination and local gravity.
        """
        self._ensure_config_state()
        data = struct.pack('!ddd', lat, lon, alt)
        self.write_ack(MID.SetLatLonAlt, data)

    def GetAvailableScenarios(self):
        """Get the available XKF scenarios on the device."""
        self._ensure_config_state()
        data = self.write_ack(MID.ReqAvailableScenarios)
        scenarios = []
        try:
            for i in range(len(data) / 22):
                scenario_type, version, label = \
                    struct.unpack('!BB20s', data[22 * i:22 * (i + 1)])
                scenarios.append((scenario_type, version, label.strip()))
            # available XKF scenarios
            self.scenarios = scenarios
        except struct.error:
            raise MTException("could not parse the available XKF scenarios.")
        return scenarios

    def GetCurrentScenario(self):
        """Get the ID of the currently used XKF scenario."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetCurrentScenario)
        _, self.scenario_id = struct.unpack('!BB', data)  # version, id
        return self.scenario_id

    def SetCurrentScenario(self, scenario_id):
        """Set the XKF scenario to use."""
        self._ensure_config_state()
        data = struct.pack('!BB', 0, scenario_id)  # version, id
        self.write_ack(MID.SetCurrentScenario, data)

    # New names in mk5
    GetAvailableFilterProfiles = GetAvailableScenarios
    GetFilterProfile = GetCurrentScenario
    SetFilterProfile = SetCurrentScenario

    def GetGnssPlatform(self):
        """Get the current GNSS navigation filter settings."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetGnssPlatform)
        platform, = struct.unpack('!H', data)
        return platform

    def SetGnssPlatform(self, platform):
        """Set the GNSS navigation filter settings."""
        self._ensure_config_state()
        data = struct.pack('!H', platform)
        self.write_ack(MID.SetGnssPlatform, data)

    def ResetOrientation(self, code):
        """Reset the orientation.

        Code can take several values:
            0x0000: store current settings (only in config mode),
            0x0001: heading reset (NOT supported by MTi-G),
            0x0003: object reset.
        """
        data = struct.pack('!H', code)
        self.write_ack(MID.ResetOrientation, data)

    def SetNoRotation(self, duration):
        """Initiate the "no rotation" procedure to estimate gyro biases."""
        self._ensure_measurement_state()
        data = struct.pack('!H', duration)
        self.write_ack(MID.SetNoRotation, data)

    def GetUTCTime(self):
        """Get UTC time from device."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetUTCTime)
        ns, year, month, day, hour, minute, second, flag = \
            struct.unpack('!IHBBBBBB', data)
        return (ns, year, month, day, hour, minute, second, flag)

    def SetUTCTime(self, ns, year, month, day, hour, minute, second, flag):
        """Set UTC time on the device."""
        self._ensure_config_state()
        data = struct.pack('!IHBBBBBB', ns, year, month, day, hour, minute,
                           second, flag)  # no clue what setting flag can mean
        self.write_ack(MID.SetUTCTime, data)

    def AdjustUTCTime(self, ticks):
        """Adjust UTC Time of device using correction ticks (0.1 ms)."""
        self._ensure_config_state()
        data = struct.pack('!i', ticks)
        self.write(MID.AdjustUTCTime, data)  # no ack mentioned in the doc

    def IccCommand(self, command):
        """Command of In-run Compass Calibration (ICC)."""
        if command not in (0, 1, 2, 3):
            raise MTException("unknown ICC command 0x%02X" % command)
        cmd_data = struct.pack('!B', command)
        res_data = self.write_ack(MID.IccCommand, cmd_data)
        cmd_ack = struct.unpack('!B', res_data[:1])
        payload = res_data[1:]
        if cmd_ack != command:
            raise MTException("expected ack of command 0x%02X; got 0x%02X "
                              "instead" % (command, cmd_ack))
        if cmd_ack == 0:
            return
        elif cmd_ack == 1:
            ddt_value, dimension, status = struct.unpack('!fBB', payload)
            return ddt_value, dimension, status
        elif cmd_ack == 2:
            return
        elif cmd_ack == 3:
            state = struct.unpack('!B', payload)
            return state

    ############################################################
    # High-level utility functions
    ############################################################
    def configure_legacy(self, mode, settings, period=None, skipfactor=None):
        """Configure the mode and settings of the MT device in legacy mode."""
        try:
            # switch mark IV devices to legacy mode
            self.SetOutputConfiguration([(0x0000, 0)])
        except MTErrorMessage as e:
            if e.code == 0x04:
                # mark III device
                pass
            else:
                raise
        except MTException as e:
            if self.verbose:
                print("no ack received while switching from MTData2 to MTData.")
            pass  # no ack???
        self.SetOutputMode(mode)
        self.SetOutputSettings(settings)
        if period is not None:
            self.SetPeriod(period)
        if skipfactor is not None:
            self.SetOutputSkipFactor(skipfactor)
        self.GetConfiguration()

    def auto_config_legacy(self):
        """Read configuration from device in legacy mode."""
        self.GetConfiguration()
        return self.mode, self.settings, self.length

    def read_measurement(self, mode=None, settings=None):
        self._ensure_measurement_state()
        # getting data
        # data = self.read_data_msg()
        mid, data = self.read_msg()
        if mid == MID.MTData:
            print("mtdata1")
            return self.parse_MTData(data, mode, settings)
        elif mid == MID.MTData2:
            print("mtdata2")
            return self.parse_MTData2(data)
        else:
            raise MTException("unknown data message: mid=0x%02X (%s)." %
                              (mid, getMIDName(mid)))

    def parse_MTData2(self, data):
        # Functions to parse each type of packet
        def parse_temperature(data_id, content, ffmt):
            o = {}
            if (data_id & 0x00F0) == 0x10:  # Temperature
                o['Temp'], = struct.unpack('!' + ffmt, content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_timestamp(data_id, content, ffmt):
            print("-------------here we do parse_timestamp")
            o = {}
            if (data_id & 0x00F0) == 0x10:  # UTC Time
                o['ns'], o['Year'], o['Month'], o['Day'], o['Hour'], \
                o['Minute'], o['Second'], o['Flags'] = \
                    struct.unpack('!LHBBBBBB', content)
            elif (data_id & 0x00F0) == 0x20:  # Packet Counter
                o['PacketCounter'], = struct.unpack('!H', content)
            elif (data_id & 0x00F0) == 0x30:  # Integer Time of Week
                o['TimeOfWeek'], = struct.unpack('!L', content)
            elif (data_id & 0x00F0) == 0x40:  # GPS Age  # deprecated
                o['gpsAge'], = struct.unpack('!B', content)
            elif (data_id & 0x00F0) == 0x50:  # Pressure Age  # deprecated
                o['pressureAge'], = struct.unpack('!B', content)
            elif (data_id & 0x00F0) == 0x60:  # Sample Time Fine
                o['SampleTimeFine'], = struct.unpack('!L', content)
            elif (data_id & 0x00F0) == 0x70:  # Sample Time Coarse
                o['SampleTimeCoarse'], = struct.unpack('!L', content)
            elif (data_id & 0x00F0) == 0x80:  # Frame Range
                o['startFrame'], o['endFrame'] = struct.unpack('!HH', content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_orientation_data(data_id, content, ffmt):
            print("-------------here we do parse_orientation_data")
            o = {}
            if (data_id & 0x000C) == 0x00:  # ENU
                o['frame'] = 'ENU'
            elif (data_id & 0x000C) == 0x04:  # NED
                o['frame'] = 'NED'
            elif (data_id & 0x000C) == 0x08:  # NWU
                o['frame'] = 'NWU'
            if (data_id & 0x00F0) == 0x10:  # Quaternion
                o['Q0'], o['Q1'], o['Q2'], o['Q3'] = struct.unpack('!' + 4 * ffmt,
                                                                   content)
            elif (data_id & 0x00F0) == 0x20:  # Rotation Matrix
                o['a'], o['b'], o['c'], o['d'], o['e'], o['f'], o['g'], \
                o['h'], o['i'] = struct.unpack('!' + 9 * ffmt, content)
            elif (data_id & 0x00F0) == 0x30:  # Euler Angles
                o['Roll'], o['Pitch'], o['Yaw'] = struct.unpack('!' + 3 * ffmt,
                                                                content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_pressure(data_id, content, ffmt):
            o = {}
            if (data_id & 0x00F0) == 0x10:  # Baro pressure
                o['Pressure'], = struct.unpack('!L', content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_acceleration(data_id, content, ffmt):
            print("-------------here we do parse_acceleration")
            o = {}
            if (data_id & 0x000C) == 0x00:  # ENU
                o['frame'] = 'ENU'
            elif (data_id & 0x000C) == 0x04:  # NED
                o['frame'] = 'NED'
            elif (data_id & 0x000C) == 0x08:  # NWU
                o['frame'] = 'NWU'
            if (data_id & 0x00F0) == 0x10:  # Delta V
                o['Delta v.x'], o['Delta v.y'], o['Delta v.z'] = \
                    struct.unpack('!' + 3 * ffmt, content)
            elif (data_id & 0x00F0) == 0x20:  # Acceleration
                o['accX'], o['accY'], o['accZ'] = \
                    struct.unpack('!' + 3 * ffmt, content)
            elif (data_id & 0x00F0) == 0x30:  # Free Acceleration
                o['freeAccX'], o['freeAccY'], o['freeAccZ'] = \
                    struct.unpack('!' + 3 * ffmt, content)
            elif (data_id & 0x00F0) == 0x40:  # AccelerationHR
                o['accX'], o['accY'], o['accZ'] = \
                    struct.unpack('!' + 3 * ffmt, content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_position(data_id, content, ffmt):
            o = {}
            if (data_id & 0x000C) == 0x00:  # ENU
                o['frame'] = 'ENU'
            elif (data_id & 0x000C) == 0x04:  # NED
                o['frame'] = 'NED'
            elif (data_id & 0x000C) == 0x08:  # NWU
                o['frame'] = 'NWU'
            if (data_id & 0x00F0) == 0x10:  # Altitude MSL  # deprecated
                o['altMsl'], = struct.unpack('!' + ffmt, content)
            elif (data_id & 0x00F0) == 0x20:  # Altitude Ellipsoid
                o['altEllipsoid'], = struct.unpack('!' + ffmt, content)
            elif (data_id & 0x00F0) == 0x30:  # Position ECEF
                o['ecefX'], o['ecefY'], o['ecefZ'] = \
                    struct.unpack('!' + 3 * ffmt, content)
            elif (data_id & 0x00F0) == 0x40:  # LatLon
                o['lat'], o['lon'] = struct.unpack('!' + 2 * ffmt, content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_GNSS(data_id, content, ffmt):
            o = {}
            if (data_id & 0x00F0) == 0x10:  # GNSS PVT data
                o['itow'], o['year'], o['month'], o['day'], o['hour'], \
                o['min'], o['sec'], o['valid'], o['tAcc'], o['nano'], \
                o['fixtype'], o['flags'], o['numSV'], o['lon'], o['lat'], \
                o['height'], o['hMSL'], o['hAcc'], o['vAcc'], o['velN'], \
                o['velE'], o['velD'], o['gSpeed'], o['headMot'], \
                o['sAcc'], o['headAcc'], o['headVeh'], o['gdop'], \
                o['pdop'], o['tdop'], o['vdop'], o['hdop'], o['ndop'], \
                o['edop'] = \
                    struct.unpack('!IHBBBBBBIiBBBxiiiiIIiiiiiIIiHHHHHHH',
                                  content)
                # scaling correction
                o['lon'] *= 1e-7
                o['lat'] *= 1e-7
                o['headMot'] *= 1e-5
                o['headVeh'] *= 1e-5
                o['gdop'] *= 0.01
                o['pdop'] *= 0.01
                o['tdop'] *= 0.01
                o['vdop'] *= 0.01
                o['hdop'] *= 0.01
                o['ndop'] *= 0.01
                o['edop'] *= 0.01
            elif (data_id & 0x00F0) == 0x20:  # GNSS satellites info
                o['iTOW'], o['numSvs'] = struct.unpack('!LBxxx', content[:8])
                svs = []
                ch = {}
                for i in range(o['numSvs']):
                    ch['gnssId'], ch['svId'], ch['cno'], ch['flags'] = \
                        struct.unpack('!BBBB', content[8 + 4 * i:12 + 4 * i])
                    svs.append(ch)
                o['svs'] = svs
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_angular_velocity(data_id, content, ffmt):
            print("-------------here we do parse_angular_velocity")
            o = {}
            if (data_id & 0x000C) == 0x00:  # ENU
                o['frame'] = 'ENU'
            elif (data_id & 0x000C) == 0x04:  # NED
                o['frame'] = 'NED'
            elif (data_id & 0x000C) == 0x08:  # NWU
                o['frame'] = 'NWU'
            if (data_id & 0x00F0) == 0x20:  # Rate of Turn
                o['gyrX'], o['gyrY'], o['gyrZ'] = \
                    struct.unpack('!' + 3 * ffmt, content)
            elif (data_id & 0x00F0) == 0x30:  # Delta Q
                o['Delta q0'], o['Delta q1'], o['Delta q2'], o['Delta q3'] = \
                    struct.unpack('!' + 4 * ffmt, content)
            elif (data_id & 0x00F0) == 0x40:  # RateOfTurnHR
                o['gyrX'], o['gyrY'], o['gyrZ'] = \
                    struct.unpack('!' + 3 * ffmt, content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_GPS(data_id, content, ffmt):
            o = {}
            if (data_id & 0x00F0) == 0x30:  # DOP
                o['iTOW'], g, p, t, v, h, n, e = \
                    struct.unpack('!LHHHHHHH', content)
                o['gDOP'], o['pDOP'], o['tDOP'], o['vDOP'], o['hDOP'], \
                o['nDOP'], o['eDOP'] = 0.01 * g, 0.01 * p, 0.01 * t, \
                                       0.01 * v, 0.01 * h, 0.01 * n, 0.01 * e
            elif (data_id & 0x00F0) == 0x40:  # SOL
                o['iTOW'], o['fTOW'], o['Week'], o['gpsFix'], o['Flags'], \
                o['ecefX'], o['ecefY'], o['ecefZ'], o['pAcc'], \
                o['ecefVX'], o['ecefVY'], o['ecefVZ'], o['sAcc'], \
                o['pDOP'], o['numSV'] = \
                    struct.unpack('!LlhBBlllLlllLHxBx', content)
                # scaling correction
                o['pDOP'] *= 0.01
            elif (data_id & 0x00F0) == 0x80:  # Time UTC
                o['iTOW'], o['tAcc'], o['nano'], o['year'], o['month'], \
                o['day'], o['hour'], o['min'], o['sec'], o['valid'] = \
                    struct.unpack('!LLlHBBBBBB', content)
            elif (data_id & 0x00F0) == 0xA0:  # SV Info
                o['iTOW'], o['numCh'] = struct.unpack('!LBxxx', content[:8])
                channels = []
                ch = {}
                for i in range(o['numCh']):
                    ch['chn'], ch['svid'], ch['flags'], ch['quality'], \
                    ch['cno'], ch['elev'], ch['azim'], ch['prRes'] = \
                        struct.unpack('!BBBBBbhl', content[8 + 12 * i:20 + 12 * i])
                    channels.append(ch)
                o['channels'] = channels
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_SCR(data_id, content, ffmt):
            o = {}
            if (data_id & 0x00F0) == 0x10:  # ACC+GYR+MAG+Temperature
                o['accX'], o['accY'], o['accZ'], o['gyrX'], o['gyrY'], \
                o['gyrZ'], o['magX'], o['magY'], o['magZ'], o['Temp'] = \
                    struct.unpack("!9Hh", content)
            elif (data_id & 0x00F0) == 0x20:  # Gyro Temperature
                o['tempGyrX'], o['tempGyrY'], o['tempGyrZ'] = \
                    struct.unpack("!hhh", content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_analog_in(data_id, content, ffmt):  # deprecated
            o = {}
            if (data_id & 0x00F0) == 0x10:  # Analog In 1
                o['analogIn1'], = struct.unpack("!H", content)
            elif (data_id & 0x00F0) == 0x20:  # Analog In 2
                o['analogIn2'], = struct.unpack("!H", content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_magnetic(data_id, content, ffmt):
            o = {}
            if (data_id & 0x000C) == 0x00:  # ENU
                o['frame'] = 'ENU'
            elif (data_id & 0x000C) == 0x04:  # NED
                o['frame'] = 'NED'
            elif (data_id & 0x000C) == 0x08:  # NWU
                o['frame'] = 'NWU'
            if (data_id & 0x00F0) == 0x20:  # Magnetic Field
                o['magX'], o['magY'], o['magZ'] = \
                    struct.unpack("!3" + ffmt, content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_velocity(data_id, content, ffmt):
            o = {}
            if (data_id & 0x000C) == 0x00:  # ENU
                o['frame'] = 'ENU'
            elif (data_id & 0x000C) == 0x04:  # NED
                o['frame'] = 'NED'
            elif (data_id & 0x000C) == 0x08:  # NWU
                o['frame'] = 'NWU'
            if (data_id & 0x00F0) == 0x10:  # Velocity XYZ
                o['velX'], o['velY'], o['velZ'] = \
                    struct.unpack("!3" + ffmt, content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_status(data_id, content, ffmt):
            print("-------------here we do parse_status")
            o = {}
            if (data_id & 0x00F0) == 0x10:  # Status Byte
                o['StatusByte'], = struct.unpack("!B", content)
            elif (data_id & 0x00F0) == 0x20:  # Status Word
                o['StatusWord'], = struct.unpack("!L", content)
            elif (data_id & 0x00F0) == 0x40:  # RSSI  # deprecated
                o['RSSI'], = struct.unpack("!b", content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        # data object
        output = {}
        while data:
            try:
                data_id, size = struct.unpack('!HB', data[:3])
                if (data_id & 0x0003) == 0x3:
                    float_format = 'd'
                elif (data_id & 0x0003) == 0x0:
                    float_format = 'f'
                else:
                    raise MTException("fixed point precision not supported.")
                content = data[3:3 + size]
                data = data[3 + size:]
                group = data_id & 0xF800
                ffmt = float_format
                if group == XDIGroup.Temperature:
                    output.setdefault('Temperature', {}).update(
                        parse_temperature(data_id, content, ffmt))
                elif group == XDIGroup.Timestamp:
                    output.setdefault('Timestamp', {}).update(
                        parse_timestamp(data_id, content, ffmt))
                elif group == XDIGroup.OrientationData:
                    output.setdefault('Orientation Data', {}).update(
                        parse_orientation_data(data_id, content, ffmt))
                    print("222222222222222222222222222222222")
                    print(output)
                elif group == XDIGroup.Pressure:
                    output.setdefault('Pressure', {}).update(
                        parse_pressure(data_id, content, ffmt))
                elif group == XDIGroup.Acceleration:
                    output.setdefault('Acceleration', {}).update(
                        parse_acceleration(data_id, content, ffmt))
                elif group == XDIGroup.Position:
                    output.setdefault('Position', {}).update(
                        parse_position(data_id, content, ffmt))
                elif group == XDIGroup.GNSS:
                    output.setdefault('GNSS', {}).update(
                        parse_GNSS(data_id, content, ffmt))
                elif group == XDIGroup.AngularVelocity:
                    output.setdefault('Angular Velocity', {}).update(
                        parse_angular_velocity(data_id, content, ffmt))
                elif group == XDIGroup.GPS:
                    output.setdefault('GPS', {}).update(
                        parse_GPS(data_id, content, ffmt))
                elif group == XDIGroup.SensorComponentReadout:
                    output.setdefault('SCR', {}).update(
                        parse_SCR(data_id, content, ffmt))
                elif group == XDIGroup.AnalogIn:  # deprecated
                    output.setdefault('Analog In', {}).update(
                        parse_analog_in(data_id, content, ffmt))
                elif group == XDIGroup.Magnetic:
                    output.setdefault('Magnetic', {}).update(
                        parse_magnetic(data_id, content, ffmt))
                elif group == XDIGroup.Velocity:
                    output.setdefault('Velocity', {}).update(
                        parse_velocity(data_id, content, ffmt))
                elif group == XDIGroup.Status:
                    output.setdefault('Status', {}).update(
                        parse_status(data_id, content, ffmt))
                else:
                    raise MTException("unknown XDI group: 0x%04X." % group)
            except struct.error:
                raise MTException("couldn't parse MTData2 message (data_id: "
                                  "0x%04X, size: %d)." % (data_id, size))
            print("here output")
            print(output)
        return output

    def parse_MTData(self, data, mode=None, settings=None):
        """Read and parse a legacy measurement packet."""
        # getting mode
        if mode is None:
            mode = self.mode
        if settings is None:
            settings = self.settings
        # data object
        output = {}
        try:
            # raw IMU first
            if mode & OutputMode.RAW:
                o = {}
                o['accX'], o['accY'], o['accZ'], o['gyrX'], o['gyrY'], \
                o['gyrZ'], o['magX'], o['magY'], o['magZ'], o['temp'] = \
                    struct.unpack('!10H', data[:20])
                data = data[20:]
                output['RAW'] = o
            # raw GPS second
            if mode & OutputMode.RAWGPS:
                o = {}
                o['Press'], o['bPrs'], o['ITOW'], o['LAT'], o['LON'], \
                o['ALT'], o['VEL_N'], o['VEL_E'], o['VEL_D'], o['Hacc'], \
                o['Vacc'], o['Sacc'], o['bGPS'] = \
                    struct.unpack('!HBI6i3IB', data[:44])
                data = data[44:]
                output['RAWGPS'] = o
            # temperature
            if mode & OutputMode.Temp:
                temp, = struct.unpack('!f', data[:4])
                data = data[4:]
                output['Temp'] = temp
            # calibrated data
            if mode & OutputMode.Calib:
                o = {}
                if (settings & OutputSettings.Coordinates_NED):
                    o['frame'] = 'NED'
                else:
                    o['frame'] = 'ENU'
                if not (settings & OutputSettings.CalibMode_GyrMag):
                    o['accX'], o['accY'], o['accZ'] = struct.unpack('!3f',
                                                                    data[:12])
                    data = data[12:]
                if not (settings & OutputSettings.CalibMode_AccMag):
                    o['gyrX'], o['gyrY'], o['gyrZ'] = struct.unpack('!3f',
                                                                    data[:12])
                    data = data[12:]
                if not (settings & OutputSettings.CalibMode_AccGyr):
                    o['magX'], o['magY'], o['magZ'] = struct.unpack('!3f',
                                                                    data[:12])
                    data = data[12:]
                output['Calib'] = o
            # orientation
            if mode & OutputMode.Orient:
                o = {}
                if (settings & OutputSettings.Coordinates_NED):
                    o['frame'] = 'NED'
                else:
                    o['frame'] = 'ENU'
                if settings & OutputSettings.OrientMode_Euler:
                    o['roll'], o['pitch'], o['yaw'] = struct.unpack('!3f',
                                                                    data[:12])
                    data = data[12:]
                elif settings & OutputSettings.OrientMode_Matrix:
                    a, b, c, d, e, f, g, h, i = struct.unpack('!9f',
                                                              data[:36])
                    data = data[36:]
                    o['matrix'] = ((a, b, c), (d, e, f), (g, h, i))
                else:  # OutputSettings.OrientMode_Quaternion:
                    q0, q1, q2, q3 = struct.unpack('!4f', data[:16])
                    data = data[16:]
                    o['quaternion'] = (q0, q1, q2, q3)
                output['Orient'] = o
            # auxiliary
            if mode & OutputMode.Auxiliary:
                o = {}
                if not (settings & OutputSettings.AuxiliaryMode_NoAIN1):
                    o['Ain_1'], = struct.unpack('!H', data[:2])
                    data = data[2:]
                if not (settings & OutputSettings.AuxiliaryMode_NoAIN2):
                    o['Ain_2'], = struct.unpack('!H', data[:2])
                    data = data[2:]
                output['Auxiliary'] = o
            # position
            if mode & OutputMode.Position:
                o = {}
                o['Lat'], o['Lon'], o['Alt'] = struct.unpack('!3f', data[:12])
                data = data[12:]
                output['Pos'] = o
            # velocity
            if mode & OutputMode.Velocity:
                o = {}
                if (settings & OutputSettings.Coordinates_NED):
                    o['frame'] = 'NED'
                else:
                    o['frame'] = 'ENU'
                o['Vel_X'], o['Vel_Y'], o['Vel_Z'] = struct.unpack('!3f',
                                                                   data[:12])
                data = data[12:]
                output['Vel'] = o
            # status
            if mode & OutputMode.Status:
                status, = struct.unpack('!B', data[:1])
                data = data[1:]
                output['Stat'] = status
            # sample counter
            if settings & OutputSettings.Timestamp_SampleCnt:
                TS, = struct.unpack('!H', data[:2])
                data = data[2:]
                output['Sample'] = TS
            # UTC time
            if settings & OutputSettings.Timestamp_UTCTime:
                o = {}
                o['ns'], o['Year'], o['Month'], o['Day'], o['Hour'], \
                o['Minute'], o['Second'], o['Flags'] = struct.unpack(
                    '!ihbbbbb', data[:12])
                data = data[12:]
                output['Timestamp'] = o
            # TODO at that point data should be empty
        except struct.error as e:
            raise MTException("could not parse MTData message.")
        if data != '':
            raise MTException("could not parse MTData message (too long).")
        return output

    def ChangeBaudrate(self, baudrate):
        """Change the baudrate, reset the device and reopen communication."""
        brid = Baudrates.get_BRID(baudrate)
        self.SetBaudrate(brid)
        self.Reset()
        # self.device.flush()
        self.device.baudrate = baudrate
        # self.device.flush()
        time.sleep(0.01)
        self.read_msg()
        self.write_msg(MID.WakeUpAck)


################################################################
# Auto detect port
################################################################
def find_devices(timeout=0.002, verbose=False, initial_wait=0.1):
    mtdev_list = []
    for port in glob.glob("/dev/tty*S*"):
        if verbose:
            print("Trying '%s'" % port)
        try:
            br = find_baudrate(port, timeout, verbose, initial_wait)
            if br:
                mtdev_list.append((port, br))
        except MTException:
            pass
    return mtdev_list


################################################################
# Auto detect baudrate
################################################################
def find_baudrate(port, timeout=0.002, verbose=False, initial_wait=0.1):
    baudrates = (115200, 460800, 921600, 230400, 57600, 38400, 19200, 9600)
    for br in baudrates:
        if verbose:
            print("Trying %d bd:" % br,
                  sys.stdout.flush())
        try:
            mt = MTDevice(port, br, timeout=timeout, verbose=verbose,
                          initial_wait=initial_wait)
        except serial.SerialException:
            if verbose:
                print("fail: unable to open device.")
            raise MTException("unable to open %s" % port)
        try:
            mt.GoToConfig()
            mt.GoToMeasurement()
            if verbose:
                print("ok.")
            return br
        except MTException:
            if verbose:
                print("fail.")


################################################################
# Documentation for stand alone usage
################################################################
def usage():
    print("""MT device driver.)
Usage:
    ./mtdevice.py [commands] [opts]

Commands:
    -h, --help
        Print this help and quit.
    -r, --reset
        Reset device to factory defaults.
    -a, --change-baudrate=NEW_BAUD
        Change baudrate from BAUD (see below) to NEW_BAUD.
    -c, --configure=OUTPUT
        Configure the device (see OUTPUT description below).
    -e, --echo
        Print MTData. It is the default if no other command is supplied.
    -i, --inspect
        Print current MT device configuration.
    -x, --xkf-scenario=ID
        Change the current XKF scenario.
    -l, --legacy-configure
        Configure the device in legacy mode (needs MODE and SETTINGS arguments
        below).
    -v, --verbose
        Verbose output.
    -y, --synchronization=settings (see below)
        Configure the synchronization settings of each sync line (see below).
    -u, --utc-time=time (see below)
        Set the UTC time buffer of the device.
    -g, --gnss-platform=platform
        Change the GNSS navigation filter settings (check the documentation).
    -o, --option-flags=flags (see below)
        Set the option flags.
    -j, --icc-command=command (see below)
        Send command to the In-run Compass Calibration.

Generic options:
    -d, --device=DEV
        Serial interface of the device (default: /dev/ttyUSB0). If 'auto', then
        all serial ports are tested at all baudrates and the first
        suitable device is used.
    -b, --baudrate=BAUD
        Baudrate of serial interface (default: 115200). If 0, then all
        rates are tried until a suitable one is found.
    -t, --timeout=TIMEOUT
        Timeout of serial communication in second (default: 0.002).
    -w, --initial-wait=WAIT
        Initial wait to allow device to be ready in second (default: 0.1).

Configuration option:
    OUTPUT
        The format is a sequence of "<group><type><frequency>?<format>?"
        separated by commas.
        The frequency and format are optional.
        The groups and types can be:
            t  temperature (max frequency: 1 Hz):
                tt  temperature
            i  timestamp (max frequency: 2000 Hz):
                iu  UTC time
                ip  packet counter
                ii  Integer Time of the Week (ITOW)
                if  sample time fine
                ic  sample time coarse
                ir  frame range
            o  orientation data (max frequency: 400 Hz):
                oq  quaternion
                om  rotation matrix
                oe  Euler angles
            b  pressure (max frequency: 50 Hz):
                bp  baro pressure
            a  acceleration (max frequency: 2000 Hz (see documentation)):
                ad  delta v
                aa  acceleration
                af  free acceleration
                ah  acceleration HR (max frequency 1000 Hz)
            p  position (max frequency: 400 Hz):
                pa  altitude ellipsoid
                pp  position ECEF
                pl  latitude longitude
            n  GNSS (max frequency: 4 Hz):
                np  GNSS PVT data
                ns  GNSS satellites info
            w  angular velocity (max frequency: 2000 Hz (see documentation)):
                wr  rate of turn
                wd  delta q
                wh  rate of turn HR (max frequency 1000 Hz)
            g  GPS (max frequency: 4 Hz):
                gd  DOP
                gs  SOL
                gu  time UTC
                gi  SV info
            r  Sensor Component Readout (max frequency: 2000 Hz):
                rr  ACC, GYR, MAG, temperature
                rt  Gyro temperatures
            m  Magnetic (max frequency: 100 Hz):
                mf  magnetic Field
            v  Velocity (max frequency: 400 Hz):
                vv  velocity XYZ
            s  Status (max frequency: 2000 Hz):
                sb  status byte
                sw  status word
        Frequency is specified in decimal and is assumed to be the maximum
        frequency if it is omitted.
        Format is a combination of the precision for real valued numbers and
        coordinate system:
            precision:
                f  single precision floating point number (32-bit) (default)
                d  double precision floating point number (64-bit)
            coordinate system:
                e  East-North-Up (default)
                n  North-East-Down
                w  North-West-Up
        Examples:
            The default configuration for the MTi-1/10/100 IMUs can be
            specified either as:
                "wd,ad,mf,ip,if,sw"
            or
                "wd2000fe,ad2000fe,mf100fe,ip2000,if2000,sw2000"
            For getting quaternion orientation in float with sample time:
                "oq400fw,if2000"
            For longitude, latitude, altitude and orientation (on MTi-G-700):
                "pl400fe,pa400fe,oq400fe"

Synchronization settings:
    The format follows the xsens protocol documentation. All fields are
    required and separated by commas.
    Note: The entire synchronization buffer is wiped every time a new one
          is set, so it is necessary to specify the settings of multiple
          lines at once.
    It also possible to clear the synchronization with the argument "clear"

        Function (see manual for details):
             3 Trigger indication
             4 Interval Transition Measurement
             8 SendLatest
             9 ClockBiasEstimation
            11 StartSampling
        Line (manual for details):
            0 ClockIn
            1 GPSClockIn (only available for 700/710)
            2 Input Line (SyncIn)
            4 SyncOut
            5 ExtTimepulseIn (only available for 700/710)
            6 Software (only available for SendLatest with ReqData message)
        Polarity:
            1 Positive pulse/ Rising edge
            2 Negative pulse/ Falling edge
            3 Both/ Toggle
        Trigger Type:
            0 multiple times
            1 once
        Skip First (unsigned_int):
            Number of initial events to skip before taking actions
        Skip Factor (unsigned_int):
            Number of events to skip before taking action again
            Ignored with ReqData.
        Pulse Width (unsigned_int):
            Ignored for SyncIn.
            For SyncOut, the width of the generated pulse in 100 microseconds
            unit. Ignored for Toggle pulses.
        Delay:
            Delay after receiving a sync pulse to taking action,
            100 microseconds units, range [0...600000]
        Clock Period:
            Reference clock period in milliseconds for ClockBiasEstimation
        Offset:
            Offset from event to pulse generation.
            100 microseconds unit, range [-30000...+30000]

    Examples:
        For changing the sync setting of the SyncIn line to trigger indication
        with rising edge, one time triggering and no skipping and delay. Enter
        the settings as:
            "3,2,1,1,0,0,0,0"

        Note a number is still in the place for pulse width despite it being
        ignored.

        To set multiple lines at once:
        ./mtdevice.py -y 3,2,1,0,0,0,0,0 -y 9,0,1,0,0,0,10,0

        To clear the synchronization settings of MTi
        ./mtdevice.py -y clear

UTC time settings:
    There are two ways to set the UTCtime for the MTi.
    Option #1: set MTi to the current UTC time based on local system time with
               the option 'now'
    Option #2: set MTi to a specified UTC time
        The time fields are set as follows:
            year: range [1999,2099]
            month: range [1,12]
            day: day of the month, range [1,31]
            hour: hour of the day, range [0,23]
            min: minute of the hour, range [0,59]
            sec: second of the minute, range [0,59]
            ns: nanosecond of the second, range [0,1000000000]
            flag:
                1: Valid Time of Week
                2: Valid Week Number
                4: valid UTC
            Note: the flag is ignored for --utc-time as it is set by the device
                  itself when connected to a GPS

    Examples:
        Set UTC time for the device:
        ./mtdevice.py -u now
        ./mtdevice.py -u 1999,1,1,0,0,0,0,0

GNSS platform settings:
    Only for MTi-G-700/710 with firmware>=1.7.
    The following two platform settings are listed in the documentation:
        0:  Portable
        8:  Airbone <4g
    Check the XSens documentation before changing anything.

Option flags:
    Several flags can be set or cleared.
    0x00000001  DisableAutoStore: when set, configuration changes are not saved
                    in non-volatile memory (only MTi-1 series)
    0x00000002  DisableAutoMeasurement: when set, device will stay in Config
                    Mode upon start up (only MTi-1 series)
    0x00000004  EnableBeidou: when set, enable Beidou and disable GLONASS (only
                    MTi-G-710)
    0x00000010  EnableAHS: enable Active Heading Stabilization (overrides
                    magnetic reference)
    0x00000080  EnableInRunCompassCalibration: doc is unclear
    The flags provided must be a pair of ored values: the first for flags to be
    set the second for the flags to be cleared.
    Examples:
        Only set DisableAutoStore and DisableAutoMeasurement flags:
            ./mtdevice.py -o 0x03,0x00
        Disable AHS (clear EnableAHS flag):
            ./mtdevice.py -o 0x00,0x10
        Set DisableAutoStore and clear DisableAutoMeasurement:
            ./mtdevice.py -o 0x02,0x01

In-run Compass Calibration commands:
    The idea of ICC is to record magnetic field data during so-called
    representative motion in order to better calibrate the magnetometer and
    improve the fusion.
    Typical usage would be to issue the start command, then move the device
    for some time then issue the stop command. If parameters are acceptable,
    these can be stored using the store command.
    Commands:
        00: Start representative motion
        01: Stop representative motion; return ddt, dimension, and status.
        02: Store ICC parameters
        03: Get representative motion state; return 1 if active
    Check the documentation for more details.

Legacy options:
    -m, --output-mode=MODE
        Legacy mode of the device to select the information to output.
        This is required for 'legacy-configure' command.
        MODE can be either the mode value in hexadecimal, decimal or
        binary form, or a string composed of the following characters
        (in any order):
            t  temperature, [0x0001]
            c  calibrated data, [0x0002]
            o  orientation data, [0x0004]
            a  auxiliary data, [0x0008]
            p  position data (requires MTi-G), [0x0010]
            v  velocity data (requires MTi-G), [0x0020]
            s  status data, [0x0800]
            g  raw GPS mode (requires MTi-G), [0x1000]
            r  raw (incompatible with others except raw GPS), [0x4000]
        For example, use "--output-mode=so" to have status and
        orientation data.
    -s, --output-settings=SETTINGS
        Legacy settings of the device. This is required for 'legacy-configure'
        command.
        SETTINGS can be either the settings value in hexadecimal,
        decimal or binary form, or a string composed of the following
        characters (in any order):
            t  sample count (excludes 'n')
            n  no sample count (excludes 't')
            u  UTC time
            q  orientation in quaternion (excludes 'e' and 'm')
            e  orientation in Euler angles (excludes 'm' and 'q')
            m  orientation in matrix (excludes 'q' and 'e')
            A  acceleration in calibrated data
            G  rate of turn in calibrated data
            M  magnetic field in calibrated data
            i  only analog input 1 (excludes 'j')
            j  only analog input 2 (excludes 'i')
            N  North-East-Down instead of default: X North Z up
        For example, use "--output-settings=tqMAG" for all calibrated
        data, sample counter and orientation in quaternion.
    -p, --period=PERIOD
        Sampling period in (1/115200) seconds (default: 1152).
        Minimum is 225 (1.95 ms, 512 Hz), maximum is 1152
        (10.0 ms, 100 Hz).
        Note that for legacy devices it is the period at which sampling occurs,
        not the period at which messages are sent (see below).

Deprecated options:
    -f, --deprecated-skip-factor=SKIPFACTOR
        Only for mark III devices.
        Number of samples to skip before sending MTData message
        (default: 0).
        The frequency at which MTData message is send is:
            115200/(PERIOD * (SKIPFACTOR + 1))
        If the value is 0xffff, no data is send unless a ReqData request
        is made.
""")


################################################################
# Main function
################################################################
def main():
    # parse command line
    shopts = 'hra:c:eild:b:m:s:p:f:x:vy:u:g:o:j:t:w:'
    lopts = ['help', 'reset', 'change-baudrate=', 'configure=', 'echo',
             'inspect', 'legacy-configure', 'device=', 'baudrate=',
             'output-mode=', 'output-settings=', 'period=',
             'deprecated-skip-factor=', 'xkf-scenario=', 'verbose',
             'synchronization=', 'utc-time=', 'gnss-platform=',
             'option-flags=', 'icc-command=', 'timeout=', 'initial-wait=']
    try:
        opts, args = getopt.gnu_getopt(sys.argv[1:], shopts, lopts)
    except getopt.GetoptError as e:
        print(e)
        usage()
        # return 1
    # default values
    device = '/dev/ttyUSB0'
    baudrate = 115200
    timeout = 0.002
    initial_wait = 0.1
    mode = None
    settings = None
    period = None
    skipfactor = None
    new_baudrate = None
    new_xkf = None
    actions = []
    verbose = False
    sync_settings = []  # list of synchronization settings

    # filling in arguments
    for o, a in opts:
        if o in ('-h', '--help'):
            usage()
            return
        elif o in ('-r', '--reset'):
            actions.append('reset')
        elif o in ('-a', '--change-baudrate'):
            try:
                new_baudrate = int(a)
            except ValueError:
                print("change-baudrate argument must be integer.")
                return 1
            actions.append('change-baudrate')
        elif o in ('-c', '--configure'):
            output_config = get_output_config(a)
            if output_config is None:
                return 1
            actions.append('configure')
        elif o in ('-e', '--echo'):
            actions.append('echo')
        elif o in ('-i', '--inspect'):
            actions.append('inspect')
        elif o in ('-l', '--legacy-configure'):
            actions.append('legacy-configure')
        elif o in ('-x', '--xkf-scenario'):
            try:
                new_xkf = int(a)
            except ValueError:
                print("xkf-scenario argument must be integer.")
                return 1
            actions.append('xkf-scenario')
        elif o in ('-y', '--synchronization'):
            new_sync_settings = get_synchronization_settings(a)
            if new_sync_settings is None:
                return 1
            sync_settings.append(new_sync_settings)
            actions.append('synchronization')
        elif o in ('-u', '--setUTCtime'):
            UTCtime_settings = get_UTCtime(a)
            if UTCtime_settings is None:
                return 1
            actions.append('setUTCtime')
        elif o in ('-d', '--device'):
            device = a
        elif o in ('-b', '--baudrate'):
            try:
                baudrate = int(a)
            except ValueError:
                print("baudrate argument must be integer.")
                return 1
        elif o in ('-m', '--output-mode'):
            mode = get_mode(a)
            if mode is None:
                return 1
        elif o in ('-s', '--output-settings'):
            settings = get_settings(a)
            if settings is None:
                return 1
        elif o in ('-p', '--period'):
            try:
                period = int(a)
            except ValueError:
                print("period argument must be integer.")
                return 1
        elif o in ('-f', '--deprecated-skip-factor'):
            try:
                skipfactor = int(a)
            except ValueError:
                print("skip-factor argument must be integer.")
                return 1
        elif o in ('-v', '--verbose'):
            verbose = True
        elif o in ('-g', '--gnss-platform'):
            platform = get_gnss_platform(a)
            if platform is None:
                return 1
            actions.append('gnss-platform')
        elif o in ('-o', '--option-flags'):
            flag_tuple = get_option_flags(a)
            if flag_tuple is None:
                return 1
            actions.append('option-flags')
        elif o in ('-j', '--icc-command'):
            icc_command = get_icc_command(a)
            if icc_command is None:
                return 1
            actions.append('icc-command')
        elif o in ('-t', '--timeout'):
            try:
                timeout = float(a)
            except ValueError:
                print("timeout argument must be a floating number.")
                return 1
        elif o in ('-w', '--initial-wait'):
            try:
                initial_wait = float(a)
            except ValueError:
                print("initial-wait argument must be a floating number.")
                return 1

    # if nothing else: echo
    if len(actions) == 0:
        actions.append('echo')
    try:
        if device == 'auto':
            devs = find_devices(timeout=timeout, verbose=verbose,
                                initial_wait=initial_wait)
            if devs:
                print("Detected devices:", "".join('\n\t%s @ %d' % (d, p)
                                                   for d, p in devs))
                print("Using %s @ %d" % devs[0])
                device, baudrate = devs[0]
            else:
                print("No suitable device found.")
                return 1
        # find baudrate
        if not baudrate:
            baudrate = find_baudrate(device, timeout=timeout, verbose=verbose,
                                     initial_wait=initial_wait)
        if not baudrate:
            print("No suitable baudrate found.")
            return 1
        # open device
        try:
            mt = MTDevice(device, baudrate, timeout=timeout, verbose=verbose,
                          initial_wait=initial_wait)
        except serial.SerialException:
            raise MTException("unable to open %s" % device)
        # execute actions
        if 'inspect' in actions:
            inspect(mt, device, baudrate)
        if 'change-baudrate' in actions:
            print("Changing baudrate from %d to %d:") % (baudrate,
                                                         new_baudrate),
            sys.stdout.flush()
            mt.ChangeBaudrate(new_baudrate)
            print(" Ok")  # should we test that it was actually ok?
        if 'reset' in actions:
            print("Restoring factory defaults",
                  sys.stdout.flush())
            mt.RestoreFactoryDefaults()
            print(" Ok")  # should we test that it was actually ok?
        if 'configure' in actions:
            print("Changing output configuration",
                  sys.stdout.flush())
            mt.SetOutputConfiguration(output_config)
            print(" Ok")  # should we test that it was actually ok?
        if 'synchronization' in actions:
            print("Changing synchronization settings",
                  sys.stdout.flush())
            mt.SetSyncSettings(sync_settings)
            print(" Ok")  # should we test that it was actually ok?
        if 'setUTCtime' in actions:
            print("Setting UTC time in the device",
                  sys.stdout.flush())
            mt.SetUTCTime(UTCtime_settings[6],
                          UTCtime_settings[0],
                          UTCtime_settings[1],
                          UTCtime_settings[2],
                          UTCtime_settings[3],
                          UTCtime_settings[4],
                          UTCtime_settings[5],
                          UTCtime_settings[7])
            print(" Ok")  # should we test that it was actually ok?
        if 'gnss-platform' in actions:
            print("Setting GNSS platform",
                  sys.stdout.flush())
            mt.SetGnssPlatform(platform)
            print(" Ok")  # should we test that it was actually ok?
        if 'option-flags' in actions:
            print("Setting option flags",
                  sys.stdout.flush())
            mt.SetOptionFlags(*flag_tuple)
            print(" Ok")  # should we test that it was actually ok?
        if 'icc-command' in actions:
            icc_command_names = {
                0: 'start representative motion',
                1: 'stop representative motion',
                2: 'store ICC results',
                3: 'representative motion state'}
            print("Sending ICC command 0x%02X (%s):" % (
                icc_command, icc_command_names[icc_command]),
                  sys.stdout.flush())
            res = mt.IccCommand(icc_command)
            if icc_command == 0x00:
                print(" Ok")  # should we test that it was actually ok?
            elif icc_command == 0x01:
                print(res)
            elif icc_command == 0x02:
                print(" Ok")  # should we test that it was actually ok?
            elif icc_command == 0x03:
                res_string = {0: 'representative motion inactive',
                              1: 'representation motion active'}
                print("0x02X (%s)" % (res, res_string.get(res, 'unknown')))
        if 'legacy-configure' in actions:
            if mode is None:
                print("output-mode is require to configure the device in " \
                      "legacy mode.")
                return 1
            if settings is None:
                print("output-settings is required to configure the device " \
                      "in legacy mode.")
                return 1
            print("Configuring in legacy mode",
                  sys.stdout.flush())
            mt.configure_legacy(mode, settings, period, skipfactor)
            print(" Ok")  # should we test it was actually ok?
        if 'xkf-scenario' in actions:
            print("Changing XKF scenario",
                  sys.stdout.flush())
            mt.SetCurrentScenario(new_xkf)
            print("Ok")
        if 'echo' in actions:
            # if (mode is None) or (settings is None):
            #     mode, settings, length = mt.auto_config()
            #     print mode, settings, length
            try:
                while True:
                    print(mt.read_measurement(mode, settings))
            except KeyboardInterrupt:
                pass
    except MTErrorMessage as e:
        print("MTErrorMessage:", e)
    except MTException as e:
        print("MTException:", e)


def inspect(mt, device, baudrate):
    """Inspection."""

    def config_fmt(config):
        """Hexadecimal configuration."""
        return '[%s]' % ', '.join('(0x%04X, %d)' % (mode, freq)
                                  for (mode, freq) in config)

    def hex_fmt(size=4):
        """Factory for hexadecimal representation formatter."""
        fmt = '0x%%0%dX' % (2 * size)

        def f(value):
            """Hexadecimal representation."""
            # length of string is twice the size of the value (in bytes)
            return fmt % value

        return f

    def sync_fmt(settings):
        """Synchronization settings: N*12 bytes"""
        return '[%s]' % ', '.join('(0x%02X, 0x%02X, 0x%02X, 0x%02X,'
                                  ' 0x%04X, 0x%04X, 0x%04X, 0x%04X)' % s
                                  for s in settings)

    def try_message(m, f, formater=None, *args, **kwargs):
        print('  %s ' % m),
        try:
            if formater is not None:
                print(formater(f(*args, **kwargs)))
            else:
                pprint.pprint(f(*args, **kwargs), indent=4)
        except MTTimeoutException as e:
            print('timeout: might be unsupported by your device.')
        except MTErrorMessage as e:
            if e.code == 0x04:
                print('message unsupported by your device.')
            else:
                raise e

    print("Device: %s at %d Bd:" % (device, baudrate))
    try_message("device ID:", mt.GetDeviceID, hex_fmt(4))
    try_message("product code:", mt.GetProductCode)
    try_message("hardware version:", mt.GetHardwareVersion)
    try_message("firmware revision:", mt.GetFirmwareRev)
    try_message("baudrate:", mt.GetBaudrate)
    try_message("error mode:", mt.GetErrorMode, hex_fmt(2))
    try_message("option flags:", mt.GetOptionFlags, hex_fmt(4))
    try_message("location ID:", mt.GetLocationID, hex_fmt(2))
    try_message("transmit delay:", mt.GetTransmitDelay)
    try_message("synchronization settings:", mt.GetSyncSettings, sync_fmt)
    try_message("general configuration:", mt.GetConfiguration)
    try_message("output configuration (mark IV devices):",
                mt.GetOutputConfiguration, config_fmt)
    try_message("string output type:", mt.GetStringOutputType)
    try_message("period:", mt.GetPeriod)
    try_message("alignment rotation sensor:", mt.GetAlignmentRotation,
                parameter=0)
    try_message("alignment rotation local:", mt.GetAlignmentRotation,
                parameter=1)
    try_message("output mode:", mt.GetOutputMode, hex_fmt(2))
    try_message("extended output mode:", mt.GetExtOutputMode, hex_fmt(2))
    try_message("output settings:", mt.GetOutputSettings, hex_fmt(4))
    try_message("GPS coordinates (lat, lon, alt):", mt.GetLatLonAlt)
    try_message("GNSS platform:", mt.GetGnssPlatform)
    try_message("available scenarios:", mt.GetAvailableScenarios)
    try_message("current scenario ID:", mt.GetCurrentScenario)
    try_message("UTC time:", mt.GetUTCTime)


def get_output_config(config_arg):
    """Parse the mark IV output configuration argument."""
    # code and max frequency
    code_dict = {
        'tt': (0x0810, 1),
        'iu': (0x1010, 2000),
        'ip': (0x1020, 2000),
        'ii': (0x1030, 2000),
        'if': (0x1060, 2000),
        'ic': (0x1070, 2000),
        'ir': (0x1080, 2000),
        'oq': (0x2010, 400),
        'om': (0x2020, 400),
        'oe': (0x2030, 400),
        'bp': (0x3010, 50),
        'ad': (0x4010, 2000),
        'aa': (0x4020, 2000),
        'af': (0x4030, 2000),
        'ah': (0x4040, 1000),
        'pa': (0x5020, 400),
        'pp': (0x5030, 400),
        'pl': (0x5040, 400),
        'np': (0x7010, 4),
        'ns': (0x7020, 4),
        'wr': (0x8020, 2000),
        'wd': (0x8030, 2000),
        'wh': (0x8040, 1000),
        'gd': (0x8830, 4),
        'gs': (0x8840, 4),
        'gu': (0x8880, 4),
        'gi': (0x88A0, 4),
        'rr': (0xA010, 2000),
        'rt': (0xA020, 2000),
        'mf': (0xC020, 100),
        'vv': (0xD010, 400),
        'sb': (0xE010, 2000),
        'sw': (0xE020, 2000)
    }
    # format flags
    format_dict = {'f': 0x00, 'd': 0x03, 'e': 0x00, 'n': 0x04, 'w': 0x08}
    config_re = re.compile('([a-z]{2})(\d+)?([fdenw])?([fdnew])?')
    output_configuration = []
    try:
        for item in config_arg.split(','):
            group, frequency, fmt1, fmt2 = config_re.findall(item.lower())[0]
            code, max_freq = code_dict[group]
            if fmt1 in format_dict:
                code |= format_dict[fmt1]
            if fmt2 in format_dict:
                code |= format_dict[fmt2]
            if frequency:
                frequency = min(max_freq, int(frequency))
            else:
                frequency = max_freq
            output_configuration.append((code, frequency))
        return output_configuration
    except (IndexError, KeyError):
        print('could not parse output specification "%s"' % item)
        return


def get_mode(arg):
    """Parse command line output-mode argument."""
    try:  # decimal
        mode = int(arg)
        return mode
    except ValueError:
        pass
    if arg[0] == '0':
        try:  # binary
            mode = int(arg, 2)
            return mode
        except ValueError:
            pass
        try:  # hexadecimal
            mode = int(arg, 16)
            return mode
        except ValueError:
            pass
    # string mode specification
    mode = 0
    for c in arg:
        if c == 't':
            mode |= OutputMode.Temp
        elif c == 'c':
            mode |= OutputMode.Calib
        elif c == 'o':
            mode |= OutputMode.Orient
        elif c == 'a':
            mode |= OutputMode.Auxiliary
        elif c == 'p':
            mode |= OutputMode.Position
        elif c == 'v':
            mode |= OutputMode.Velocity
        elif c == 's':
            mode |= OutputMode.Status
        elif c == 'g':
            mode |= OutputMode.RAWGPS
        elif c == 'r':
            mode |= OutputMode.RAW
        else:
            print("Unknown output-mode specifier: '%s'" % c)
            return
    return mode


def get_settings(arg):
    """Parse command line output-settings argument."""
    try:  # decimal
        settings = int(arg)
        return settings
    except ValueError:
        pass
    if arg[0] == '0':
        try:  # binary
            settings = int(arg, 2)
            return settings
        except ValueError:
            pass
        try:  # hexadecimal
            settings = int(arg, 16)
            return settings
        except ValueError:
            pass
    # strings settings specification
    timestamp = 0
    orient_mode = 0
    calib_mode = OutputSettings.CalibMode_Mask
    NED = 0
    for c in arg:
        if c == 't':
            timestamp = OutputSettings.Timestamp_SampleCnt
        elif c == 'n':
            timestamp = OutputSettings.Timestamp_None
        elif c == 'u':
            timestamp |= OutputSettings.Timestamp_UTCTime
        elif c == 'q':
            orient_mode = OutputSettings.OrientMode_Quaternion
        elif c == 'e':
            orient_mode = OutputSettings.OrientMode_Euler
        elif c == 'm':
            orient_mode = OutputSettings.OrientMode_Matrix
        elif c == 'A':
            calib_mode &= OutputSettings.CalibMode_Acc
        elif c == 'G':
            calib_mode &= OutputSettings.CalibMode_Gyr
        elif c == 'M':
            calib_mode &= OutputSettings.CalibMode_Mag
        elif c == 'i':
            calib_mode &= OutputSettings.AuxiliaryMode_NoAIN2
        elif c == 'j':
            calib_mode &= OutputSettings.AuxiliaryMode_NoAIN1
        elif c == 'N':
            NED = OutputSettings.Coordinates_NED
        else:
            print("Unknown output-settings specifier: '%s'" % c)
            return
    settings = timestamp | orient_mode | calib_mode | NED
    return settings


def get_synchronization_settings(arg):
    """Parse command line synchronization-settings argument."""
    if arg == "clear":
        sync_settings = [0, 0, 0, 0, 0, 0, 0, 0]
        return sync_settings
    else:
        # Parse each field from the argument
        sync_settings = arg.split(',')
        try:
            # convert string to int
            sync_settings = tuple([int(i) for i in sync_settings])
        except ValueError:
            print("Synchronization sync_settings must be integers.")
            return
        # check synchronization sync_settings
        if sync_settings[0] in (3, 4, 8, 9, 11) and \
                sync_settings[1] in (0, 1, 2, 4, 5, 6) and \
                sync_settings[2] in (1, 2, 3) and \
                sync_settings[3] in (0, 1):
            return sync_settings
        else:
            print("Invalid synchronization settings.")
            return


def get_UTCtime(arg):
    """Parse command line UTC time specification."""
    # If argument is now, fill the time settings with the current time
    # else fill the time settings with the specified time
    if arg == "now":
        timestamp = datetime.datetime.utcnow()  # use datetime to get microsec
        time_settings = []
        time_settings.append(timestamp.year)
        time_settings.append(timestamp.month)
        time_settings.append(timestamp.day)
        time_settings.append(timestamp.hour)
        time_settings.append(timestamp.minute)
        time_settings.append(timestamp.second)
        time_settings.append(timestamp.microsecond * 1000)  # *1000 to get ns
        time_settings.append(0)  # default flag to 0
        return time_settings
    else:
        # Parse each field from the argument
        time_settings = arg.split(',')
        try:
            time_settings = [int(i) for i in time_settings]
        except ValueError:
            print("UTCtime settings must be integers.")
            return

        # check UTCtime settings
        if 1999 <= time_settings[0] <= 2099 and \
                1 <= time_settings[1] <= 12 and \
                1 <= time_settings[2] <= 31 and \
                0 <= time_settings[3] <= 23 and \
                0 <= time_settings[4] <= 59 and \
                0 <= time_settings[5] <= 59 and \
                0 <= time_settings[6] <= 1000000000:
            return time_settings
        else:
            print("Invalid UTCtime settings.")
            return


def get_gnss_platform(arg):
    """Parse and check command line GNSS platform argument."""
    try:
        platform = int(arg)
    except ValueError:
        print("GNSS platform must be an integer.")
        return
    if platform in (0, 8):
        return platform
    else:
        print("Invalid GNSS platform argument (excepted 0 or 8).")
        return


def get_option_flags(arg):
    """Parse and check command line option flags argument."""
    try:
        set_flag, clear_flag = map(lambda s: int(s.strip(), base=0),
                                   arg.split(','))
        return (set_flag, clear_flag)
    except ValueError:
        print('incorrect option flags specification (expected a pair of ' \
              'values)')
        return


def get_icc_command(arg):
    """Parse and check ICC command argument."""
    try:
        icc_command = int(arg, base=0)
        if icc_command not in range(4):
            raise ValueError
        return icc_command
    except ValueError:
        print('invalid ICC command "%s"; expected 0, 1, 2, or 3.' % arg)
        return


if __name__ == '__main__':
    main()
