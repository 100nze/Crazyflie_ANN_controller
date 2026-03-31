#!/usr/bin/env python3
"""
--------------------------------------------------------------------
  ENTER      ARM  (start heartbeat →  takeoff sequence) / LAND (start landing sequence)
  SPACE      DISARM (Emergency stop)

  1          MODE_HOVER        (hover at takeoff origin)
  2          MODE_NORMAL       (position teleop)
  3          MODE_FIGURE_EIGHT (autonomous figure-8)

  W / S      forward / backward   (x ±0.1 m)
  A / D      left / right         (y ±0.1 m)
  P / L      up / down            (z ±0.1 m)

  H          print help
  CTRL-C     quit (disarm first)
---------------------------------------------------------------------
"""
import sys
import time
import threading
import select
import termios
import tty
from shutil import get_terminal_size
from collections import deque
from cflib.crtp.crtpstack import CRTPPacket
from cflib.utils.reset_estimator import reset_estimator

HEARTBEAT_HZ = 20
HEARTBEAT_PORT = 9
TELEOP_HZ = 20
POS_STEP = 0.1

MODE_NORMAL = 0
MODE_HOVER = 1
MODE_FIGURE_EIGHT = 2
MODE_LANDING = 3
MODE_STOP = 4

MODE_NAMES = {
    MODE_NORMAL: "NORMAL",
    MODE_HOVER: "HOVER",
    MODE_FIGURE_EIGHT: "FIGURE-8",
    MODE_LANDING: "LANDING",
    MODE_STOP: "STOP",
}

MOVEMENT_MAP = {
    'w': (0, POS_STEP), 's': (0, -POS_STEP),
    'a': (1, POS_STEP), 'd': (1, -POS_STEP),
    'p': (2, POS_STEP), 'l': (2, -POS_STEP),
}


class RawTerminal:
    """Handles raw terminal input without buffering."""
    def __enter__(self):
        self._old = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, *_):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old)

    @staticmethod
    def key_available():
        return select.select([sys.stdin], [], [], 0)[0]

    @staticmethod
    def read_key():
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            if RawTerminal.key_available():
                ch2 = sys.stdin.read(1)
                if ch2 == '[' and RawTerminal.key_available():
                    ch3 = sys.stdin.read(1)
                    return {'A': 'UP', 'B': 'DOWN', 'C': 'RIGHT', 'D': 'LEFT'}.get(ch3, None)
            return 'ESC'
        return ch

class HeartbeatThread(threading.Thread):
    """Sends a CRTP packet at HEARTBEAT_HZ to keep the control link active."""
    def __init__(self, cf):
        super().__init__(daemon=True)
        self.cf = cf
        self.active = threading.Event()
        self.stop_evt = threading.Event()

    def run(self):
        dt = 1.0 / HEARTBEAT_HZ
        while not self.stop_evt.is_set():
            if self.active.is_set():
                pk = CRTPPacket()
                pk.port = HEARTBEAT_PORT
                pk.data = b''
                self.cf.send_packet(pk)
            self.stop_evt.wait(dt)

    def arm(self):
        self.active.set()

    def disarm(self):
        self.active.clear()

    def is_armed(self):
        return self.active.is_set()

    def shutdown(self):
        self.active.clear()
        self.stop_evt.set()

def print_help():
    print("\n── Keybindings ─────────────────────────────")
    print("  ENTER        Takeoff / Landing")
    print("  SPACE        DISARM (Emergency)")
    print("  1            MODE_HOVER        (hover)")
    print("  2            MODE_NORMAL       (position teleop/ trajectory execution)")
    print("  3            MODE_FIGURE_EIGHT")
    print("  W / S        forward / backward (±0.1 m)")
    print("  A / D        left / right       (±0.1 m)")
    print("  P / L        up / down          (±0.1 m)")
    print("  H            print this help")
    print("  CTRL-C       quit (disarms automatically)")
    print("────────────────────────────────────────────\n")

class ConsoleInterceptor:
    """Capture stdout lines and mirror them into HUD queue."""
    def __init__(self, message_queue):
        self._queue = message_queue
        self._partial = ""

    def write(self, text):
        self._partial += text
        while True:
            nl_idx = self._partial.find('\n')
            cr_idx = self._partial.find('\r')

            if nl_idx == -1 and cr_idx == -1:
                break

            if nl_idx == -1:
                sep_idx = cr_idx
            elif cr_idx == -1:
                sep_idx = nl_idx
            else:
                sep_idx = min(nl_idx, cr_idx)

            line = self._partial[:sep_idx]
            self._partial = self._partial[sep_idx + 1:]
            msg = line.strip()
            if msg:
                self._queue.append(msg)

    def flush(self):
        pass


class Controller:
    HUD_LOG_ROWS = 50
    def __init__(self, cf, logger, default_height=0.3):
        self.cf = cf
        self.logger = logger
        self.heartbeat = HeartbeatThread(self.cf)
        self.mode = MODE_HOVER
        self.pos_cmd = [0.0, 0.0, 0.0]
        self.fw_msgs = deque()
        self.log_history = deque(maxlen=self.HUD_LOG_ROWS)
        self.default_height = default_height
        self.hud_lines_drawn = 0
        self.count = 0
        self.sum = 0.0
        self.sq_sum = 0.0
        self._orig_stdout = None
        self._interceptor = None
        self._term_out = sys.__stdout__ if hasattr(sys, '__stdout__') and sys.__stdout__ else sys.stdout

    def run(self):
        self.heartbeat.start()
        self.set_mode(self.mode)
        dt = 1.0 / TELEOP_HZ
        self.install_console_interceptor()

        with RawTerminal():
            try:
                while True:
                    t0 = time.monotonic()
                    snap = self.logger.snapshot()

                    self.process_keys(snap)
                    self.is_landing_complete(snap)
                    self.send_setpoints()
                    self.render_hud(snap)

                    elapsed = time.monotonic() - t0
                    if elapsed < dt:
                        time.sleep(dt - elapsed)

            except KeyboardInterrupt:
                raise
            finally:
                self.shutdown()

    def install_console_interceptor(self):
        self._orig_stdout = sys.stdout
        self._interceptor = ConsoleInterceptor(self.fw_msgs)
        sys.stdout = self._interceptor

    def uninstall_console_interceptor(self):
        if self._orig_stdout is not None:
            sys.stdout = self._orig_stdout
        self._orig_stdout = None
        self._interceptor = None

    # Flight logic 

    def takeoff(self):
        self.set_mode(self.mode)
        self.pos_cmd = [0.0, 0.0, self.default_height]
        self.count = 0
        self.sum = 0.0
        self.sq_sum = 0.0
        # Let reset_estimator() print directly to terminal while it blocks.
        self.uninstall_console_interceptor()
        try:
            reset_estimator(self.cf)
        finally:
            self.install_console_interceptor()

        self.heartbeat.arm()

    def land(self):
        self.set_mode(MODE_LANDING)
        # ignore low-level setpoints so HLC can execute landing
        self.cf.commander.send_notify_setpoint_stop()

    def set_mode(self, target_mode):
        self.mode = target_mode
        self.cf.param.set_value('ann.mode', str(self.mode))
        self.logger.mode = self.mode 

    def is_landing_complete(self, snap):
        """If landing is complete disarms the drone and sets it to hover mode."""
        if self.mode == MODE_LANDING and self.heartbeat.is_armed() and not snap['motors_active']:
            # Drone has finished landing and disarmed itself
            self.heartbeat.disarm()
            self.mode = MODE_HOVER
            self.pos_cmd = [0.0, 0.0, 0.0]

    def send_setpoints(self):
        if self.heartbeat.is_armed() and self.mode != MODE_LANDING:
            self.cf.commander.send_position_setpoint(self.pos_cmd[0], self.pos_cmd[1], self.pos_cmd[2], 0.0)

    def shutdown(self):
        self.uninstall_console_interceptor()
        self.heartbeat.disarm()
        self.heartbeat.shutdown()
        self.heartbeat.join(timeout=1.0)
        self.cf.commander.send_position_setpoint(0.0, 0.0, 0.0, 0.0)
        time.sleep(0.3)
        self.teardown_scroll_region()

    def process_keys(self,snap):
        while RawTerminal.key_available():
            key = RawTerminal.read_key()
            if not key:
                continue
                
            key_lower = key.lower()

            if key in ('\n', '\r'):
                self.land() if self.heartbeat.is_armed() else self.takeoff()
            elif key == ' ':
                self.set_mode(MODE_STOP)
                self.heartbeat.disarm()
                self.mode = MODE_HOVER  
            elif key in ('1', '2', '3'):
                mode_map = {'1': MODE_HOVER, '2': MODE_NORMAL, '3': MODE_FIGURE_EIGHT}
                target_mode = mode_map[key]
                if target_mode == MODE_NORMAL and self.mode != MODE_NORMAL:
                    self.pos_cmd[0] = snap['target'][0]
                    self.pos_cmd[1] = snap['target'][1]
                    self.pos_cmd[2] = snap['target'][2]

                self.set_mode(mode_map[key])
            elif key == '0':
                self.pos_cmd = [0.0, 0.0, self.default_height]
            elif key_lower in MOVEMENT_MAP:
                axis, step = MOVEMENT_MAP[key_lower]
                self.pos_cmd[axis] += step
            elif key_lower == 'h':
                print_help()

    def teardown_scroll_region(self):
        self._term_out.write('Controller disabled. Shutting down...\n')
        self._term_out.flush()

    def render_hud(self, snap):
        while self.fw_msgs:
            self.log_history.append(self.fw_msgs.popleft())

        armed_str = '\033[92m ARMED\033[0m' if snap['motors_active'] else '\033[91m○ DISARMED\033[0m'
        mode_str  = MODE_NAMES.get(self.mode, f'?({self.mode})')
        lock_status = '\033[91mLOCKED\033[0m' if snap.get('is_locked') else '\033[92mOK\033[0m'

        val = snap.get('nn_inference_us', 0.0)
        if val > 0:
            self.count += 1
            self.sum += val
            self.sq_sum += val*val

        mean = self.sum / self.count if self.count > 0 else 0.0
        var = (self.sq_sum / self.count) - (mean ** 2) if self.count > 0 else 0.0

        
        lines = [
            f"╔══════════════ ANN Controller ",
            f"║ {armed_str}   Mode: {mode_str:<15s} Supervisor: {lock_status}",
            f"║ err:  x={snap['err'][0]:+.3f}  y={snap['err'][1]:+.3f}  z={snap['err'][2]:+.3f}  m",
            f"║ rel: x={snap['rel_pos'][0]:+.3f}  y={snap['rel_pos'][1]:+.3f}  z={snap['rel_pos'][2]:+.3f}",
            f"║ tgt:  x={snap['target'][0]:+.3f}  y={snap['target'][1]:+.3f}  z={snap['target'][2]:+.3f} )" ,
            f"║ cmd:  x={self.pos_cmd[0]:+.3f}  y={self.pos_cmd[1]:+.3f}  z={self.pos_cmd[2]:+.3f} )" ,
            f"║ motor_pwm: {snap['motors'][0]:5d} {snap['motors'][1]:5d} {snap['motors'][2]:5d} {snap['motors'][3]:5d}",
            f"║ inference:{val:>7.1f}us  μ:{mean:>7.1f}  σ²:{max(0, var):>7.1f}",
            f"╚════════ ENTER=takeoff/land SPACE=emergency stop  1/2/3=mode  H=help "
        ]

        term_rows = get_terminal_size(fallback=(120, 40)).lines
        max_visible_logs = max(0, term_rows - len(lines) - 1)
        visible_logs = list(self.log_history)[-max_visible_logs:] if max_visible_logs > 0 else []
        panel_lines = lines + [f'\033[93m {msg}\033[0m' for msg in visible_logs]

        if self.hud_lines_drawn > 0:
            self._term_out.write(f'\033[{self.hud_lines_drawn}A')
            self._term_out.write('\033[J')

        for ln in panel_lines:
            self._term_out.write(f'{ln}\n')
        self._term_out.flush()

        self.hud_lines_drawn = len(panel_lines)