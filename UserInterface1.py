import tkinter as tk
import serial
import time

arduino = serial.Serial(port='COM5', baudrate=9600, timeout=1)

def send_command(command):
    arduino.write(f"{command}\n".encode())     
    time.sleep(0.1)                     
    response = arduino.readline().decode('utf-8').strip()
    result_label.config(text=response)  

def move_up():
    send_command("UP")
    print("up")

def move_down():
    print("down")
    send_command("DOWN")

def move_left():
    send_command("LEFT")
    print("left")
def move_right():
    send_command("RIGHT")
    print("right")
def stop():
    send_command("STOP")
    print("stop")

def handle_keypress(event):
    if event.keysym == "Up":
        up_button.invoke()
        #up_button.configure(background="red")
    if event.keysym == "Down":
        down_button.invoke()
        #up_button.configure(background="red")
    if event.keysym == "Right":
        right_button.invoke()
        #up_button.configure(background="red")
    if event.keysym == "Left":
        left_button.invoke()
        #up_button.configure(background="red")
    if event.keysym == "space":
        stop_button.invoke()
        #up_button.configure(background="red")



root = tk.Tk()
root.title("Robot Controller")
root.configure(bg="pink")

up_button = tk.Button(root, text="Up", font="white", command=move_up, width=10, height=2)
up_button.grid(row=0, column=1)

left_button = tk.Button(root, text="Left", font="white", command=move_left, width=10, height=2)
left_button.grid(row=1, column=0)

right_button = tk.Button(root, text="Right", font="white", command=move_right, width=10, height=2)
right_button.grid(row=1, column=2)

down_button = tk.Button(root, text="Down", font="white", command=move_down, width=10, height=2)
down_button.grid(row=2, column=1)

stop_button = tk.Button(root, text="Stop", bg="red", font="white", fg="white", command=stop, width=10, height=2)
stop_button.grid(row=1, column=1)

root.bind("<Up>", handle_keypress)
root.bind("<Down>", handle_keypress)
root.bind("<Right>", handle_keypress)
root.bind("<Left>", handle_keypress)
root.bind("<space>", handle_keypress)

# Label to display the Arduino's response
result_label = tk.Label(root, text="", width=40, height=2)
result_label.grid(row=4, column=1, pady=10)

root.mainloop()

arduino.close()
