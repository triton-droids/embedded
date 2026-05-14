from generic_read_client import GatewayClient, GenericReadClient

gw = GatewayClient("127.0.0.1", 8080)
reader = GenericReadClient(gw)

topic = "/imu/data"
msg_type = "sensor_msgs/msg/Imu"

reader.subscribe(topic, msg_type)

msg = reader.wait_for_message(topic, msg_type, timeout_sec=5.0)
print("full imu msg:", msg)

gyro_z = reader.get_field(topic, msg_type, "angular_velocity.z")
acc_x = reader.get_field(topic, msg_type, "linear_acceleration.x")

print("gyro z:", gyro_z)
print("acc x:", acc_x)