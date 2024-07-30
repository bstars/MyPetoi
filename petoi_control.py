from cProfile import label
from os import name
import sys
sys.path.append('..')

from cvxpy import pos
import serial
import time
import numpy as np
import threading
import wx



from MyPetoi.petoi_kinematics import PetoiKinematics
from MyPetoi.gaits import gaits

ser = serial.Serial(
        # port = '/dev/cu.usbserial-110',
        port='/dev/cu.usbmodem56D00029981',
        baudrate = 9600,
        xonxoff = False,
        timeout=2
)
time.sleep(2)


current_gait_name = 'stand'
current_gait = gaits['stand']
current_gait_frame_idx = 0

petoi_kinematics = PetoiKinematics(render_mode='')

lock = threading.Lock()


def switch_gait(gait_name):
    global lock, gaits, current_gait_name, current_gait, current_gait_frame_idx, petoi_kinematics
    if gait_name == current_gait_name: return
    current_gait_name = gait_name
    current_gait = gaits[gait_name]
    current_gait_frame_idx = 0

def gait_loop():
    global lock, petoi_kinematics
    lock.acquire()
    petoi_kinematics.plot_initialize()
    lock.release()

    while True:
        global gaits, current_gait_name, current_gait, current_gait_frame_idx
        lock.acquire()
        alphas, betas = petoi_kinematics.leg_ik(current_gait[current_gait_frame_idx])
        
        s = ("{:.1f} " * 8 + '\n').format(
            *(
                alphas.tolist() + betas.tolist()
            )
        )

        ser.write(s.encode('utf-8'))
        ser.flush()

        current_gait_frame_idx = (current_gait_frame_idx + 1) % len(current_gait)
        petoi_kinematics.update_gamma_h()

        lock.release()
        time.sleep(0.1)
        s = ser.readline()

def keyboard_input_loop():
    """
    input format should be 
        1. "g:walk" switch gait to walk
        2. "h:1.1" make (relative) body height 1.1
        3. "a:30" make body angle 30 degrees
    """
    while True:
        global lock, gaits, current_gait_name, current_gait, current_gait_frame_idx, petoi_kinematics
        # print('input:')
        msg = input()
        msg1, msg2 = msg[0], msg[2:]

        lock.acquire()
        if msg1 == 'g':
            if msg2 not in gaits.keys():
                print("Gait not available")
            else:
                switch_gait(msg2)
        
        elif msg1 == 'h':
            petoi_kinematics.update_gamma_h(gamma=None, h=float(msg2))
        
        elif msg1 == 'a':
            petoi_kinematics.update_gamma_h(
                gamma=np.deg2rad(float(msg2)), 
                h=None
            )
        lock.release()



class Window(wx.Frame):
    def __init__(self):
        super().__init__(parent=None)

        self.panel = wx.Panel(self)

        vbox = wx.BoxSizer(wx.VERTICAL)

        slider_angle = wx.Slider(
            self.panel, value = 0, minValue = -20, maxValue = 25,
            style = wx.SL_HORIZONTAL|wx.SL_LABELS
        ) 
        slider_angle.Bind(wx.EVT_SLIDER, self.OnScrollAngle)
        txt_angle = wx.StaticText(self.panel, label = 'Angle(degree)',style = wx.ALIGN_CENTER)
   
        # rescale [-1., 1.] to [-100, 100]
        slider_height = wx.Slider(
            self.panel, 
            value = 0, minValue = -100, maxValue = 100,
            style = wx.SL_HORIZONTAL|wx.SL_LABELS
        ) 
        slider_height.Bind(wx.EVT_SLIDER, self.OnScrollHeight)
        txt_height = wx.StaticText(self.panel, label = 'Height(mm)',style = wx.ALIGN_CENTER)

        button_stand = wx.Button(self.panel, label='stand', pos=(50,50), name='stand')
        button_stand.Bind(wx.EVT_BUTTON, self.OnClick)

        button_walk = wx.Button(self.panel, label='walk', pos=(150,50), name='walk')
        button_walk.Bind(wx.EVT_BUTTON, self.OnClick)

        button_step = wx.Button(self.panel, label='step', pos=(200,50), name='step')
        button_step.Bind(wx.EVT_BUTTON, self.OnClick)
        

        vbox.Add(button_walk)
        vbox.Add(button_stand)
        vbox.Add(button_step)
        vbox.Add(txt_angle)
        vbox.Add(slider_angle)
        vbox.Add(txt_height)
        vbox.Add(slider_height)

        self.panel.SetSizer(vbox)
        self.Show()

    def OnClick(self, e):
        widget = e.GetEventObject()
        global lock
        lock.acquire()
        switch_gait(widget.GetName())
        lock.release()

    def OnScrollHeight(self, e):
        global lock, petoi_kinematics
        val = e.GetEventObject().GetValue() / 100
        
        lock.acquire()
        petoi_kinematics.update_gamma_h(gamma=None, h=val)
        lock.release()
    
    def OnScrollAngle(self, e):
        global lock, petoi_kinematics
        val = e.GetEventObject().GetValue()
        val = np.deg2rad(val)
        
        lock.acquire()
        petoi_kinematics.update_gamma_h(gamma=val, h=None)
        lock.release()
    

def gui_input_loop():
    app = wx.App()
    window = Window()
    app.MainLoop()


if __name__ == '__main__':
    t = threading.Thread(target=gait_loop)
    t.daemon = True
    t.start()
    gui_input_loop()
    # t.join()