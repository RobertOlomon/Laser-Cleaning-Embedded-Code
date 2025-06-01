import transmitter
import threading
import time

def background_reader(serial_port):
    while True:
        try:
            line = serial_port.readline()
            if line:
                print("ESP32:", line.decode(errors='replace').strip())
        except Exception as e:
            print("Read error:", e)
            break

def main():
    tx = transmitter.Transmitter(port="COM9", baud_rate=921600, write_timeout=1, timeout=1)
    
    # Start the background reader
    reader_thread = threading.Thread(target=background_reader, args=(tx.serial,), daemon=True)
    reader_thread.start()
    
    # Main loop for sending commands
    while True:
        user_input = input("Enter command: ")
        if user_input.strip().lower() == "exit":
            print("Exiting...")
            break

        if not user_input.endswith("\0"):
            user_input += "\0"
        
        msg = transmitter.CommandMessage(user_input)
        tx.send_msg(msg)
        print(f"Sent: {user_input.strip()}")

    tx.serial.close()
    print("Serial connection closed.")

if __name__ == "__main__":
    main()
