from generic_read_client import GatewayClient, GenericReadClient

gw = GatewayClient("127.0.0.1", 8080)
reader = GenericReadClient(gw)

topic = "/joint_states"
msg_type = "sensor_msgs/msg/JointState"

reader.subscribe(topic, msg_type)

msg = reader.get_latest(topic, msg_type)
print("joint states:", msg)

names = reader.get_field(topic, msg_type, "name", default=[])
positions = reader.get_field(topic, msg_type, "position", default=[])

print("names:", names)
print("positions:", positions)