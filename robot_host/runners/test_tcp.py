import socket

# Connect to MCU
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('10.0.0.60', 3333))
sock.settimeout(5.0)

# Send a heartbeat command
import json
cmd = json.dumps({"kind": "cmd", "type": "CMD_HEARTBEAT", "seq": 1})
payload = cmd.encode('utf-8')

# Frame it
HEADER = 0xAA
MSG_CMD_JSON = 0x50
length = 1 + len(payload)
len_hi = (length >> 8) & 0xFF
len_lo = length & 0xFF
checksum = (length + MSG_CMD_JSON + sum(payload)) & 0xFF

frame = bytes([HEADER, len_hi, len_lo, MSG_CMD_JSON]) + payload + bytes([checksum])
print(f"Sending: {frame.hex()}")
sock.send(frame)

# Receive response
response = sock.recv(1024)
print(f"Received: {response.hex()}")
print(f"Decoded: {response}")

sock.close()