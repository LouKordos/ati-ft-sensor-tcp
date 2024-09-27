# Driver for ATI Force Torque Sensor via UDP

import socket
import logging
import time
import datetime
from typing import List, Tuple

BUFFER_SIZE = 1024
MESSAGE_LENGTH = 20 # bytes
HEADER_LENGTH = 2 # bytes

class ATIForceTorqueSensor:
    def __init__(self, ip: str, port=49151, enable_logging=False, loglevel=logging.INFO, timeout=0.1):
        self.ip = ip
        self.port = port
        self.connected = False
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.timeout = timeout

        self.F_bias = [0, 0, 0]
        self.T_bias = [0, 0, 0]
        self.scaling_factors_F = []
        self.scaling_factors_T = []
        self.force_units = None
        self.torque_units = None
        self.counts_per_force = None
        self.counts_per_torque = None

        self.enable_logging = enable_logging
        self.logger = logging.getLogger(__name__)
        if self.enable_logging:
            logging.basicConfig(level=loglevel)
        else:
            self.logger.addHandler(logging.NullHandler())

    def __enter__(self):
        start = time.perf_counter_ns()
        self.sock.connect((self.ip, self.port))
        self.sock.settimeout(self.timeout)
        self.connected = True
        end = time.perf_counter_ns()
        self.logger.debug(f"Socket.connect execution duration={((end-start)/1000.0):.4f}[us]")
        start = time.perf_counter_ns()
        self.force_units, self.torque_units, self.counts_per_force, self.counts_per_torque, self.scaling_factors_F, self.scaling_factors_T = self.get_calibration_info()
        end = time.perf_counter_ns()
        self.logger.debug(f"get_calibration_function() execution duration={((end-start)/1000.0):.4f}[us]")

        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        if self.connected:
            self.sock.close()

    def _construct_command(self, command_code: str) -> bytes:
        command = bytes.fromhex(command_code)
        reserved = bytes.fromhex("00" * (MESSAGE_LENGTH - len(command))) # Padding
        return command + reserved
    
    def get_calibration_info(self) -> Tuple[int, int, int, int, int, List[int], List[int]]:
        if not self.connected:
            self.logger.error("Socket is not connected yet, cannot read calibration data. It is unsafe to proceed without calibration data, throwing.")
            self.sock.close()
            raise Exception("Could not read calibration data because socket is not in connected state, it is unsafe to proceed without calibration data.")
        
        message = self._construct_command("01")
        self.logger.debug(f"calibration message={message}")

        self.sock.sendall(message)
        try:
            response_data = self.sock.recv(BUFFER_SIZE)
            
            self.logger.debug(f"len(response_data)={len(response_data)}")

            # Byte array, see ATI NetFT docs for structure
            # TODO: Refactor to use struct.unpack
            header = response_data[0:HEADER_LENGTH].hex()
            force_units = response_data[HEADER_LENGTH] # uint8
            torque_units = response_data[HEADER_LENGTH+1] # uint8
            counts_per_force = int.from_bytes(response_data[HEADER_LENGTH+2:HEADER_LENGTH+2+4], signed=True) # int32
            counts_per_torque = int.from_bytes(response_data[HEADER_LENGTH+2+4:HEADER_LENGTH+2+4+4], signed=True) # int32
            self.logger.info(f"force_units={force_units}, torque_units={torque_units}, counts_per_force={counts_per_force}, counts_per_torque={counts_per_torque}")

            # Scaling factors for F_{X,Y,Z}, then T_{X,Y,Z}
            scaling_factors = []
            SCALING_FACTOR_BYTE_OFFSET = 12
            for i in range(6):
                factor = int.from_bytes(response_data[SCALING_FACTOR_BYTE_OFFSET+i*2:SCALING_FACTOR_BYTE_OFFSET+i*2+2], signed=True)
                scaling_factors.append(factor)

            scaling_factors_F = scaling_factors[0:3]
            scaling_factors_T = scaling_factors[3:6]

            self.logger.info(f"scaling_factors_F={scaling_factors_F}\tscaling_factors_T={scaling_factors_T}")

            return force_units, torque_units, counts_per_force, counts_per_torque, scaling_factors_F, scaling_factors_T
        except socket.timeout: # Other exceptions should just be passed to caller because this is fatal.
            self.logger.error(f"Socket timeout after {self.timeout}[sec], cannot read calibration data. It is unsafe to proceed without calibration data, throwing.")
            self.sock.close()
            raise Exception(f"Could not read calibration data because of recv() timeout after {self.timeout}[sec]. It is unsafe to proceed without calibration data.")
    
    def zero_sensor(self) -> None:
        F, T = self.get_data(raw=True)
        self.F_bias = F.copy()
        self.T_bias = T.copy()

    
    def get_data(self, raw=False) -> Tuple[List[float]|None, List[float]|None]:
        """Get force and torque data from the sensor. 
        
        Args: raw (bool): If True, returns raw data without bias correction. 
        
        Returns: Tuple[List[float], List[float]]: Force and torque data, None if timeout
        """

        if not self.connected:
            self.logger.error("Socket is not connected yet, cannot read data.")
            return None, None
        
        start = time.perf_counter_ns()
        message = self._construct_command("00")

        F = [0, 0, 0] # X,Y,Z
        T = [0, 0, 0] # X,Y,Z

        end = time.perf_counter_ns()
        self.logger.debug(f"get_data() message setup execution duration={((end-start)/1000.0):.4f}[us]")

        start = time.perf_counter_ns()
        self.sock.sendall(message)
        end = time.perf_counter_ns()
        self.logger.debug(f"get_data() send execution duration={((end-start)/1000.0):.4f}[us]")
        try:
            start = time.perf_counter_ns()
            data = self.sock.recv(BUFFER_SIZE)
            end = time.perf_counter_ns()
            self.logger.debug(f"get_data() recv execution duration={((end-start)/1000.0):.4f}[us]")

            # TODO: Refactor to use struct.unpack
            start = time.perf_counter_ns()
            F_BYTE_OFFSET = 4
            for i in range(len(F)):
                # Each int16 is 2 bytes
                raw_value = int.from_bytes(data[F_BYTE_OFFSET+i*2:F_BYTE_OFFSET+i*2+2], signed=True)
                F[i] = raw_value * self.scaling_factors_F[i] / self.counts_per_force
                if not raw:
                    F[i] -= self.F_bias[i]

            end = time.perf_counter_ns()
            self.logger.debug(f"get_data() force parse execution duration={((end-start)/1000.0):.4f}[us]")

            start = time.perf_counter_ns()

            T_BYTE_OFFSET = 10
            for i in range(len(T)):
                # Each int16 is 2 bytes
                raw_value = int.from_bytes(data[T_BYTE_OFFSET+i*2:T_BYTE_OFFSET+i*2+2], signed=True)
                T[i] = raw_value * self.scaling_factors_T[i] / self.counts_per_torque
                if not raw:
                    T[i] -= self.T_bias[i]

            end = time.perf_counter_ns()
            self.logger.debug(f"get_data() torque parse execution duration={((end-start)/1000.0):.4f}[us]")

            self.logger.debug(f"Force_xyz={F}\tTorque_xyz={T}")

            return F, T
        except socket.timeout:
            self.logger.error(f"Socket timeout after {self.timeout}[sec], cannot read Force/Torque data.")
            return None, None