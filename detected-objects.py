import numpy as np
import matplotlib.pyplot as plt
from radarclass import RadarDataReader

config_file_path = 'config_files/'  # Insert your configuration file here
radar_data_reader = RadarDataReader(config_file_path)

# Configure the serial ports
port_cli = '/dev/ttyACM0'
port_data = '/dev/ttyACM1'
cli_port_baud = 115200
data_port_baud = 921600

# Open the serial ports
cli_port, data_port = radar_data_reader.serial_config(port_cli, port_data, cli_port_baud, data_port_baud)
cf = radar_data_reader.parse_config_file(num_rx_ant=4, num_tx_ant=2)

plt.figure(figsize=(6, 6))

try:
    while True:
        radar_data, frameOk = radar_data_reader.read_radar_data(data_port, cf)

        if frameOk:
            print(radar_data)
            x = radar_data["x"]
            y = radar_data["y"]

            plt.clf()

            ax = plt.subplot(111, polar=True)

            # Converting Cartesian coordinates to polar coordinates
            r = np.sqrt(x ** 2 + y ** 2)
            theta = np.arctan2(y, x)

            # Plotting the data
            ax.scatter(theta, r, color='green', s=20)

            ax.set_thetamin(0)
            ax.set_thetamax(180)

            # Setting labels
            ax.set_title('X-Y Scatter Plot for detected objects', va='bottom')
            ax.set_xlabel('Distance along lateral axis (meters)', labelpad=20)
            ax.set_ylabel('Distance along longitudinal axis (meters)', labelpad=20)

            plt.pause(0.5)

except KeyboardInterrupt:
    print("Stopping radar data reading.")

finally:
    if cli_port is not None and cli_port.is_open:
        cli_port.close()
    if data_port is not None and data_port.is_open:
        data_port.close()
