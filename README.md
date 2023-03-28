# Scan Provider
"Scan Provider" or "Driver" refers to this c++ program.

Scan provider interfaces with rplidar device using [Slamtec/rplidar_sdk](https://github.com/Slamtec/rplidar_sdk). Then sends processed data points to [SLSfEi/web-app](https://github.com/SLSfEi/web-app) backend via HTTP POST request using [libcpr/cpr](https://github.com/libcpr/cpr)

When raw data points are read. They will be corrected with linear regression to improve readings accuracy. After that, they will be serialized as CSV and sent to the [SLSfEi/web-app](https://github.com/SLSfEi/web-app) to be displayed.

# Configurations
The configuration file must be named `config.ini` and be located in the same directory as the `scan_provider` executable.
## Variable groups
Variables are divided into groups according to their functions.
### `debug`
Group | Variable | Type | Default
--- | --- | --- | --- 
debug | gen_random | boolean | 0
- When `gen_random` is set to `1`, scan provider will skip all rplidar related code and send randomly generated data points to the server.


### `connection`
Group | Variable | Type | Default
--- | --- | --- | --- 
connection | server_endpoint | string | -
- `server_endpoint` specifies the URL of the server that scan provider will send HTTP POST requests to. Must be set.


### `correction`
Group | Variable | Type | Default
--- | --- | --- | --- 
correction | offset | float | 0.0
correction | multiplier | float | 1.0


These variables are used for distance correction according to this linear regression formula.  
`corrected_distance = offset + (multiplier * raw_distance)`


### `retry`
Group | Variable | Type | Default
--- | --- | --- | --- 
retry | base_delay | integer | 300
retry | max_delay | integer | 3000


Scan provider will wait a specific amount of time before retrying when it encounters an error. With each subsequent error, the wait time will double. When there is no longer any error, the wait time will be reset to its minimum.
- `base_delay` is the minimum wait time.
- `max_delay` is the maximum wait time.


### `serial`
Group | Variable | Type | Default
--- | --- | --- | --- 
serial | port | string | -
serial | baudrate | integer | -

Variables required to establish a serial connection with rplidar device.
- `port` is the serial port of rplidar device.
- `baudrate` is the baudrate of the serial port.

## Example of `config.ini`
```
[debug]
gen_random = 0

[connection]
server_endpoint = http://127.0.0.1:8000/api/v1/scan

[correction]
offset = 0.0
multiplier = 1.0

[retry]
base_delay = 300
max_delay = 3000

[serial]
port = COM5
baudrate = 115200
```
