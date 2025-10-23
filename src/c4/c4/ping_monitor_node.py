import rclpy
from rclpy.node import Node
import subprocess
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading

class PingMonitorNode(Node):
    def __init__(self):
        super().__init__('ping_monitor_node')
        self.ip = '192.168.0.2'
        self.sma_window = 20
        self.x_vals = []
        self.y_vals = []
        self.start_time = time.time()

        # 타이머 대신 쓰레드로 Ping 수집
        self.ping_thread = threading.Thread(target=self.ping_loop)
        self.ping_thread.daemon = True
        self.ping_thread.start()

    def ping_loop(self):
        while rclpy.ok():
            ping_time = self.get_ping_time(self.ip)
            if ping_time is not None:
                current_time = time.time() - self.start_time
                self.x_vals.append(current_time)
                self.y_vals.append(ping_time)

                if len(self.x_vals) > 500:
                    self.x_vals.pop(0)
                    self.y_vals.pop(0)

            time.sleep(0.2)  # 0.2초마다 ping

    def get_ping_time(self, ip):
        try:
            result = subprocess.run(['ping', '-c', '1', ip],
                                    stdout=subprocess.PIPE,
                                    stderr=subprocess.PIPE,
                                    text=True)
            if result.returncode == 0:
                output = result.stdout
                time_ms = output.split('time=')[1].split(' ms')[0]
                return float(time_ms)
        except Exception as e:
            self.get_logger().error(f"Ping error: {e}")
        return None

    def calculate_sma(self, data):
        if len(data) < self.sma_window:
            return []
        return np.convolve(data, np.ones(self.sma_window)/self.sma_window, mode='valid')

def main(args=None):
    rclpy.init(args=args)
    node = PingMonitorNode()

    fig, ax = plt.subplots()
    line_ping, = ax.plot([], [], label='Ping(ms)')
    line_sma, = ax.plot([], [], label='SMA(ms)')
    ax.set_xlabel('Time (s)', fontsize=20)
    ax.set_ylabel('Ping (ms)', fontsize=20)
    ax.legend(fontsize=15)
    ax.set_ylim(1, 1000)  # 최소 1 이상으로 설정하여 log-scale 오류 방지
    ax.set_yscale('log')  # log scale 설정
    sma_text = ax.text(0.7, 0.9, '', transform=ax.transAxes, fontsize=20, color='red')

    def update(frame):
        x = node.x_vals
        y = node.y_vals
        sma = node.calculate_sma(y)
        line_ping.set_data(x, y)
        line_sma.set_data(x[node.sma_window - 1:], sma)

        if x:
            ax.set_xlim(max(0, x[-1] - 60), x[-1] + 1)

        if len(sma) > 0:
            sma_text.set_text(f"SMA: {sma[-1]:.2f} ms")
        else:
            sma_text.set_text("")


        return line_ping, line_sma, sma_text

    # 반드시 변수에 할당!
    ani = FuncAnimation(fig, update, interval=500)

    plt.tight_layout()
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
