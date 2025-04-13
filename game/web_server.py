import socket

ESP32_UDP_IP = "0.0.0.0" 
ESP32_UDP_PORT = 1234       

recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock.bind((ESP32_UDP_IP, ESP32_UDP_PORT))

LOCAL_UDP_IP = "127.0.0.1"
LOCAL_UDP_PORT = 12345       

send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print(f"Listening for ESP32 data on UDP {ESP32_UDP_PORT}...")
print(f"Forwarding to local UDP {LOCAL_UDP_IP}:{LOCAL_UDP_PORT}")

while True:
    data, addr = recv_sock.recvfrom(1024)
    message = data.decode()
    joystick_x, joystick_y, accelerometer_z = message.split(";")

    # print(f"Joystick: ({joystick_x}, {joystick_y}), Accelerometer: {accelerometer_z}")

    x = int(joystick_x) // 100
    y = int(joystick_y) // 100
    z = float(accelerometer_z)
    
    x_shift = 0
    jump_y = 0
    jump_z = 0

    if not (27 <= x <= 29):
        x_offset = x - 28
        x_shift = int((x_offset / 28.5) * 10)

    if abs(y - 28) > 10:
        jump_y = 1

    if z < 7.5:
        jump_z = 1

    info = [str(x_shift), str(jump_y), str(jump_z)]
    delimeter = ';'
    send_message = delimeter.join(info).encode("utf-8")
    
    send_sock.sendto(send_message, (LOCAL_UDP_IP, LOCAL_UDP_PORT))