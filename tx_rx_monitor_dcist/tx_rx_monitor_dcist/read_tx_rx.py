
# TODO include this instead of duplicating in tx_rx_monitor_node.py

from pathlib import Path
import time

def timestamp() -> float:
    return time.time_ns() / 1e9

class TxRxMonitor:

    def __init__(self, iface: str):
        self.iface = iface
        base = Path(f'/sys/class/net/{self.iface}/statistics')
        self.tx_bytes_path = base / 'tx_bytes'
        self.rx_bytes_path = base / 'rx_bytes'

        assert self.tx_bytes_path.exists(), f"Path {self.tx_bytes_path} does not exist"
        assert self.rx_bytes_path.exists(), f"Path {self.rx_bytes_path} does not exist"

        self.tx_prev = self.read_tx_bytes()
        self.rx_prev = self.read_rx_bytes()
        self.timestamp_prev = timestamp()

    def read_counter(self, path: Path) -> int:
        return int(path.read_text().strip())

    def read_tx_bytes(self) -> int:
        return self.read_counter(self.tx_bytes_path)

    def read_rx_bytes(self) -> int:
        return self.read_counter(self.rx_bytes_path)

    def mk_dict(self) -> dict:
        tx_bytes = self.read_tx_bytes()
        rx_bytes = self.read_rx_bytes()
        tx_curr = tx_bytes - self.tx_prev
        rx_curr = rx_bytes - self.rx_prev
        self.tx_prev = tx_bytes
        self.rx_prev = rx_bytes
        timestamp_curr = timestamp()
        delta_t = timestamp_curr - self.timestamp_prev
        self.timestamp_prev = timestamp_curr

        return {
            'iface': self.iface,
            'timestamp': timestamp_curr,
            'delta_t': delta_t,
            'tx_bytes': tx_curr,
            'rx_bytes': rx_curr,
        }

def main():
    tx_rx = TxRxMonitor('wlan0')
    while True:
        data = tx_rx.mk_dict()
        print(data)
        time.sleep(1)

if __name__ == '__main__':
    main()