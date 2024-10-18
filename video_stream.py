import cv2
import socket
import numpy as np
import threading
import time

def send_frames(camera, connection):
    while True:
        ret, frame = camera.read()
        if not ret:
            print("Error: Could not read frame.")
            break
        _, buffer = cv2.imencode('.jpg', frame)  
        connection.sendall(buffer.tobytes()) 
        time.sleep(0.1) 

def receive_frames():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect(('127.0.0.1', 12345)) 

    while True:
        frame_data = bytearray()
        while True:
            part = client_socket.recv(4096)  
            if not part:
                break
            frame_data.extend(part)

        if frame_data:
            np_arr = np.frombuffer(frame_data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            cv2.imshow("Received Frame", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):  
                break

    client_socket.close()
    cv2.destroyAllWindows()

def main():
    camera = cv2.VideoCapture(0)  
    if not camera.isOpened():
        print("Error: Could not open camera.")
        return

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('0.0.0.0', 12345))
    server_socket.listen(1)
    print("Waiting for a connection...")
    connection, client_address = server_socket.accept() 
    print(f"Connected to {client_address}")

    sender_thread = threading.Thread(target=send_frames, args=(camera, connection))
    sender_thread.start()

    receive_frames()

    sender_thread.join()
    camera.release()
    connection.close()
    server_socket.close()

if __name__ == "__main__":
    main()
