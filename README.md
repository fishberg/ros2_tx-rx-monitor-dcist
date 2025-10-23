# TX RX Monitor

## Running
```
ros2 run tx_rx_monitor_dcist tx_rx_monitor_node

ros2 launch tx_rx_monitor_dcist tx_rx_monitor_dcist.launch.yaml launch_tx_rx:=true

ros2 topic echo /tx_rx_usage --full-length
```

## Notes
```
$cat /sys/class/net/wlan0/statistics/tx_bytes
996266526

$ cat /sys/class/net/wlan0/statistics/rx_bytes
2171752262
```