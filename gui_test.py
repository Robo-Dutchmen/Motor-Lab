import PySimpleGUI as sg
import serial

SERIAL_PORT = 'COM6'
SERIAL_BAUD = 9600


class Collect_data():
    def __inti__(self):
        self.serial = serial.Serial()

    def capture_data(self):
        msg = self.serial.readline()
        parsed = msg.decode("utf-8").replace("\r\n", "\n")

 
    def start_gui(self):

        sg.theme('DarkAmber')   # Add a little color to your windows
        # All the stuff inside your window. This is the PSG magic code compactor...
        self.layout = [  [sg.Text('Some text on Row 1')],
                    [sg.Text('Enter something on Row 2'), sg.InputText()],
                    [sg.Slider(range=(-180,180), orientation ='h', size=(34,20), default_value=85)],
                    [sg.OK(), sg.Cancel()]]

        # Create the Window
        self.window = sg.Window('Window Title', self.layout)
        # Event Loop to process "events"
        while True:             
            event, values = self.window.read()
            if event in (None, 'Cancel'):
                break
            print(values)

        self.window.close()


if __name__=="__main__":
    cd  =Collect_data()
    cd.start_gui()