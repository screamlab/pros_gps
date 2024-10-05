import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial
import time

class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps_topic', 10)
        self.ser = serial.Serial('/dev/ttyUSB0', 4800, timeout=5)  # Adjust as necessary
        self.timer = self.create_timer(1.0, self.publish_gps_data)  # Publish every second

    def publish_gps_data(self):
        line = self.ser.readline().decode(errors='ignore').strip()
        splitline = line.split(',')

        if splitline[0] == '$GNGGA':  # NMEA GGA sentence for GPS data
            # Extract raw latitude and longitude
            raw_latitude = splitline[2]
            lat_direction = splitline[3]
            raw_longitude = splitline[4]
            lon_direction = splitline[5]

            if raw_latitude and raw_longitude:
                # Convert raw NMEA coordinates to decimal degrees
                latitude = self.nmea_to_decimal(raw_latitude, lat_direction, "latitude")
                longitude = self.nmea_to_decimal(raw_longitude, lon_direction, "longitude")

                # Publish GPS data
                if latitude is not None and longitude is not None:
                    msg = NavSatFix()
                    msg.latitude = latitude
                    msg.longitude = longitude
                    msg.altitude = 0.0  # You can set the altitude if available
                    msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Published GPS: Lat {latitude}, Lon {longitude}')
                else:
                    self.get_logger().error('Failed to parse GPS data.')

    def nmea_to_decimal(self, raw_coord, direction, coord_type):
        """Convert NMEA latitude/longitude format to decimal degrees."""
        if not raw_coord or len(raw_coord) < 6:
            return None

        try:
            # Latitude (DDMM.MMMM) and Longitude (DDDMM.MMMM) handling
            if coord_type == "latitude":
                degrees = float(raw_coord[:2])
                minutes = float(raw_coord[2:]) / 60.0
            elif coord_type == "longitude":
                degrees = float(raw_coord[:3])
                minutes = float(raw_coord[3:]) / 60.0
            else:
                return None

            # Combine degrees and minutes
            decimal_degrees = degrees + minutes

            # Apply direction (South and West should be negative)
            if direction == 'S' or direction == 'W':
                decimal_degrees = -decimal_degrees

            return decimal_degrees  # Use 6 decimal places for better accuracy

        except ValueError:
            self.get_logger().error('Invalid NMEA data for conversion')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = GPSPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
