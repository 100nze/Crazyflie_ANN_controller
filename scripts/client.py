#!/usr/bin/env python3
import sys
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from logger import Logger
from controller import Controller
import os
from datetime import datetime
import subprocess 

URI = 'radio://0/50/2M/E7E7E7E7E8'
LOG_PERIOD_MS = 100

CACHE_DIR = os.path.join(os.path.dirname(__file__), 'cache')

LOG_DIR = os.path.abspath(os.path.join(__file__, '..', '..', 'flight_log'))
os.makedirs(LOG_DIR, exist_ok=True)
timestamp = datetime.now().strftime("%H-%M_%d-%m")

DEFAULT_HEIGHT = 0.8
file_name = f"flight_log_{timestamp}.csv"
csv_path = os.path.join(LOG_DIR, file_name)

def main():
    cflib.crtp.init_drivers()
    plotter_process = None 
    try:
        with SyncCrazyflie(URI, cf = Crazyflie(rw_cache = CACHE_DIR)) as scf:
            cf = scf.cf

            if scf.cf.param.get_value('deck.bcFlow2') == '1':
                print('Flow deck v2 detected')
            else:
                print('No flow deck detected! Exiting.')
                return

            cf.param.set_value('ann.takeoff_h', str(DEFAULT_HEIGHT)) 
            cf.param.set_value('ann.fig_eight_scale', '0.3')
            cf.param.set_value('ann.fig_eight_period', '5.5')


            logger = Logger(cf, log_period_ms=LOG_PERIOD_MS, csv_path=csv_path)
            controller = Controller(cf, logger, default_height=DEFAULT_HEIGHT)

            console_buf = []
            def console_cb(text):
                console_buf.append(text)
                if '\n' in text:
                    line = ''.join(console_buf).rstrip('\n')
                    controller.fw_msgs.append(line)
                    console_buf.clear()

            cf.console.receivedChar.add_callback(console_cb)
            
            plotter_process = subprocess.Popen(
                [sys.executable, 'scripts/plotter.py', csv_path, '--live'],
                start_new_session=True,
            )

            logger.start()
            try:
                controller.run()  # blocking
            except KeyboardInterrupt:
                print("\nKeyboard interrupt received")
            finally:
                logger.stop()
                if plotter_process is not None:
                    plotter_process.terminate()
                    try:
                        plotter_process.wait(timeout=2.0)
                    except subprocess.TimeoutExpired:
                        plotter_process.kill()
        
    except Exception as e:
        print(f"\nclient error: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()