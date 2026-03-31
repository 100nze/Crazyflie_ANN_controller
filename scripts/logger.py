import csv
import time
import threading
from cflib.crazyflie.log import LogConfig


class Logger:
    def __init__(self, cf, log_period_ms=100, csv_path=None):
        self.cf = cf
        self.configs = []
        self.lock = threading.Lock()
        self.csv_path = csv_path
        self.csv_file = None
        self.writer = None
        self.log_period_ms = log_period_ms
        self.header_written = False 

        self.motors_active = 0
        self.flight_start_time = None
        self.mode = 0
        self.nn_inference_us = 0.0
        self.controller_time_us = 0.0
        self.is_locked = False
        self.target = [0.0, 0.0, 0.0] 
        self.err = [0.0, 0.0, 0.0]
        self.motors = [0, 0, 0, 0] 
        self.rel_pos = [0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0] 
        self.target = [0.0, 0.0, 0.0]

    def start(self):

        lc1 = LogConfig(name='ann_status', period_in_ms=self.log_period_ms)
        lc1.add_variable('ann.motors_spinning',  'uint8_t') # motor armed ?
        lc1.add_variable('ann.err_x',  'float') # error x
        lc1.add_variable('ann.err_y',  'float') # error y
        lc1.add_variable('ann.err_z',  'float') # error z
        lc1.add_variable('supervisor.info', 'uint16_t')
        lc1.add_variable('ann.nn_time', 'float')
        lc1.add_variable('ann.ctrl_time', 'float')
        lc1.data_received_cb.add_callback(self.cb_status)
        self.cf.log.add_config(lc1)
        lc1.start()
        self.configs.append(lc1)


        lc3 = LogConfig(name='ann_pos', period_in_ms=self.log_period_ms)
        lc3.add_variable('ann.rel_x', 'float') # relative position x (from takeoff point)
        lc3.add_variable('ann.rel_y', 'float')
        lc3.add_variable('ann.rel_z', 'float')
        lc3.add_variable('ann.tgt_x', 'float') # target position x
        lc3.add_variable('ann.tgt_y', 'float')
        lc3.add_variable('ann.tgt_z', 'float')
        lc3.data_received_cb.add_callback(self.cb_pos)
        self.cf.log.add_config(lc3)
        lc3.start()
        self.configs.append(lc3)

        lc_vel = LogConfig(name='vel', period_in_ms=self.log_period_ms)
        lc_vel.add_variable('stateEstimate.vx', 'float')
        lc_vel.add_variable('stateEstimate.vy', 'float')
        lc_vel.add_variable('stateEstimate.vz', 'float')
        lc_vel.data_received_cb.add_callback(self.cb_vel)
        self.cf.log.add_config(lc_vel)
        lc_vel.start()
        self.configs.append(lc_vel)

        lc4 = LogConfig(name='motors', period_in_ms=self.log_period_ms)
        lc4.add_variable('motor.m1', 'uint16_t')
        lc4.add_variable('motor.m2', 'uint16_t')
        lc4.add_variable('motor.m3', 'uint16_t')
        lc4.add_variable('motor.m4', 'uint16_t')
        lc4.data_received_cb.add_callback(self.cb_motors)
        self.cf.log.add_config(lc4)
        lc4.start()
        self.configs.append(lc4)

    def stop(self):
        for lc in self.configs:
            lc.stop()
        if self.csv_file:
            self.csv_file.close()

    def snapshot(self):
        with self.lock:
            return {
                'motors_active':       self.motors_active,
                'err':         list(self.err),
                'motors':      list(self.motors),
                'rel_pos':     list(self.rel_pos),
                'target':      list(self.target),
                'is_locked':     self.is_locked,
                'nn_inference_us': self.nn_inference_us,
                'ctrl_time': self.controller_time_us
            }

    def cb_status(self, timestamp, data, logconf):
        with self.lock:
            prev_motors_active = self.motors_active 
            self.motors_active = data.get('ann.motors_spinning', 0)
            self.err = [data.get('ann.err_x', 0.0), 
                        data.get('ann.err_y', 0.0),
                          data.get('ann.err_z', 0.0)]
            info = data.get('supervisor.info', 0)
            self.is_locked = (info & (1 << 6)) != 0
            self.nn_inference_us = data.get('ann.nn_time', 0.0)
            self.controller_time_us = data.get('ann.ctrl_time', 0.0)

            if prev_motors_active == 0 and self.motors_active > 0:
                self.flight_start_time = time.monotonic()
            elif prev_motors_active > 0 and self.motors_active == 0:
                self.flight_start_time = None
        
        if (self.motors_active > 0) and (not self.is_locked):
            self.write_csv()
            
    def cb_motors(self, timestamp, data, logconf):
        with self.lock:
            self.motors = [data.get('motor.m1', 0),
                                data.get('motor.m2', 0),
                                data.get('motor.m3', 0),
                                data.get('motor.m4', 0)]

    def cb_pos(self, timestamp, data, logconf):
        with self.lock:
            self.rel_pos = [data.get('ann.rel_x', 0.0),
                            data.get('ann.rel_y', 0.0),
                            data.get('ann.rel_z', 0.0)]
            self.target  = [data.get('ann.tgt_x', 0.0),
                            data.get('ann.tgt_y', 0.0),
                            data.get('ann.tgt_z', 0.0)]
    def cb_vel(self, timestamp, data, logconf):
        with self.lock:
            self.vel = [data.get('stateEstimate.vx', 0.0),
                        data.get('stateEstimate.vy', 0.0),
                        data.get('stateEstimate.vz', 0.0)]


    def write_csv(self):
        with self.lock:
            if not self.header_written and self.csv_path:
                self.csv_file = open(self.csv_path, 'w', newline='')
                self.writer = csv.writer(self.csv_file)
                self.writer.writerow(['time', 'nn_inference_us','mode','err_x','err_y','err_z',
                                      'rpos_x','rpos_y','rpos_z',
                                      'tgt_x','tgt_y','tgt_z',
                                      'm1','m2','m3','m4',
                                      'vx','vy','vz'])
                self.header_written = True

            if self.writer:
                elapsed = 0.0
                if self.flight_start_time is not None:
                    elapsed = time.monotonic() - self.flight_start_time

                self.writer.writerow([elapsed, self.nn_inference_us, self.mode,
                self.err[0], self.err[1], self.err[2],
                self.rel_pos[0], self.rel_pos[1], self.rel_pos[2],
                self.target[0], self.target[1], self.target[2],
                self.motors[0], self.motors[1], self.motors[2], self.motors[3],
                self.vel[0], self.vel[1], self.vel[2]])