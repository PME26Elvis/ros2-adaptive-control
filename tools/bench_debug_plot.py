#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import numpy as np
from collections import deque

class DebugPlot(Node):
    def __init__(self, dof, nx, window=1000):
        super().__init__('debug_plot')
        self.dof = dof
        self.nx = nx
        self.buf_y = [deque(maxlen=window) for _ in range(dof)]
        self.buf_r = [deque(maxlen=window) for _ in range(dof)]
        self.sub = self.create_subscription(Float64MultiArray, 'debug', self.cb, 10)

    def cb(self, msg):
        data = msg.data
        # layout: [x(2n) | xhat(2n) | u(n) | r(n)]
        n = self.dof
        nx = self.nx
        if len(data) < (nx+nx+n+n): return
        x = np.array(data[:nx])
        r = np.array(data[nx+nx+n : nx+nx+n+n])
        # y = position = first n of x
        y = x[:n]
        for i in range(n):
            self.buf_y[i].append(y[i])
            self.buf_r[i].append(r[i])

def main():
    rclpy.init()
    # 依你的設定改這兩個數（或改成參數）
    dof, nx = 2, 4
    node = DebugPlot(dof, nx)
    try:
        for _ in range(2000):
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    # 畫圖
    fig, axes = plt.subplots(dof, 1, figsize=(6, 2.5*dof), sharex=True)
    if dof == 1: axes = [axes]
    for i in range(dof):
        axes[i].plot(list(node.buf_y[i]), label=f'y{i}')
        axes[i].plot(list(node.buf_r[i]), label=f'r{i}', linestyle='--')
        axes[i].legend(loc='best'); axes[i].grid(True)
    axes[-1].set_xlabel('samples')
    plt.tight_layout(); plt.show()

if __name__ == '__main__':
    main()
