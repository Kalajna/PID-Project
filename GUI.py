import tkinter
import serial

ser = serial.Serial(
    port="COM5",
    baudrate=115200,
    timeout=0.1,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    xonxoff=False,
    rtscts=False,
    dsrdtr=False
)

app = tkinter.Tk()
app.geometry("900x500")
app.title("Heating Resistor with PID")


def getting_temperature():
    line = ser.readline().decode().strip()
    line = str(line)
    if ser.isOpen():
        current_temp_label.configure(text=line)
        current_temp_label.after(1000, getting_temperature)


def temp_up():
    data = 'up'
    ser.write(data.encode())


def temp_down():
    data = 'down'
    ser.write(data.encode())


frame = tkinter.Frame(master=app)
frame.pack(pady=20, padx=6, fill="both", expand=True)

# Labels
main_label = tkinter.Label(master=frame, text="Heating Resistor with PID", font=("Roboto", 24))
main_label.pack(pady=12, padx=10)

current_temp_lab_label = tkinter.Label(
    master=frame,
    text="Current Temperature    Reference Temperature",
    font=("Roboto", 16)
)
current_temp_lab_label.pack(pady=30, padx=15)

current_temp_label = tkinter.Label(master=frame, font=("Roboto", 16))
current_temp_label.pack(pady=30, padx=10)

# Buttons
button_up = tkinter.Button(
    master=frame,
    text="Heat up",
    command=temp_up,
    height=3,
    width=15,
    fg="red"
)
button_up.pack(pady=12, padx=10)

button_down = tkinter.Button(
    master=frame,
    text="Heat down",
    command=temp_down,
    height=3,
    width=15,
    fg='blue'
)
button_down.pack(pady=12, padx=10)

getting_temperature()

app.mainloop()
