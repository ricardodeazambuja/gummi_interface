#!/usr/bin/env python
# -­*­- coding: utf-8 ­-*-­

from threading import Lock
import wx
import csv
import sys
import random, codecs
import time
from math import pi

import rospy
from sensor_msgs.msg import JointState


class MyFrame(wx.Frame):
    def __init__(self, parent, id, title):
        self.gui_available = False  # True when the GUI object is available
        self.received = False  # True when the first joint_states were received
        self.refreshed = True

        self.gui_lock = Lock()

        wx.Frame.__init__(self, parent, id, title, wx.DefaultPosition, (1400,700))
        panel = wx.Panel(self, -1)

        self.joints = rospy.get_param("/gummi/joints")

        rospy.init_node('GummiHMI', anonymous=False, disable_signals=True)

        self.cmdRate = rospy.Rate(60)
        self.statesRate = rospy.Rate(30)

        for name in self.joints.keys():
            self.joints[name]['text'] = wx.StaticText(panel, label=name, pos= wx.DefaultPosition)
            self.joints[name]['sldr_pos'] = wx.Slider(panel,
                                                      value = 0* (180/pi),
                                                      minValue = rospy.get_param("/gummi/"+name+"/minAngle") * (180/pi),
                                                      maxValue = rospy.get_param("/gummi/"+name+"/maxAngle") * (180/pi),
                                                      pos = wx.DefaultPosition,
                                                      size = (150, -1),
                                                      style = wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS)
            self.joints[name]['sldr_pos'].Bind(wx.EVT_SCROLL, self.OnSliderScroll)
            self.joints[name]['sldr_pos'].Enable(False)

            self.joints[name]['sldr_coc'] = wx.Slider(panel,
                                                      value = 60,
                                                      minValue = 1,
                                                      maxValue = 100,
                                                      pos = wx.DefaultPosition,
                                                      size = (150, -1),
                                                      style = wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS)

            self.joints[name]['sldr_coc'].Bind(wx.EVT_SCROLL, self.OnSliderScroll)
            self.joints[name]['sldr_coc'].Enable(False)

            self.joints[name]['vbox'] = wx.BoxSizer(wx.VERTICAL)
            self.joints[name]['vbox'].Add(self.joints[name]['text'], 1, wx.ALIGN_CENTRE)
            self.joints[name]['vbox'].Add(self.joints[name]['sldr_pos'], 1, wx.ALIGN_CENTRE)
            self.joints[name]['vbox'].Add(wx.StaticText(panel, label= "Joint angle", pos= wx.DefaultPosition),1 , wx.ALIGN_CENTRE)
            self.joints[name]['vbox'].Add(self.joints[name]['sldr_coc'], 1 , wx.ALIGN_CENTRE)
            self.joints[name]['vbox'].Add(wx.StaticText(panel, label= "Co-contraction", pos= wx.DefaultPosition), 1 , wx.ALIGN_CENTRE)


        lastbox = wx.BoxSizer(wx.VERTICAL)
        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hboxc = wx.BoxSizer(wx.HORIZONTAL)

        cb1 = wx.CheckBox(panel, label = 'Passive', pos = wx.DefaultPosition)
        cb1.Bind(wx.EVT_CHECKBOX,self.onChecked)

        btn1 = wx.Button(panel, 12, 'Save')
        wx.EVT_BUTTON(btn1, 12, self.OnSave)

        btn2 = wx.Button(panel, 12, 'Refresh')
        wx.EVT_BUTTON(btn2, 12, self.OnRefresh)


        for name in self.joints.keys():
            hbox.Add(self.joints[name]['vbox'], 1 , wx.ALIGN_CENTRE)

        lastbox.Add(hbox, 1 , wx.ALIGN_CENTRE)
        lastbox.Add(hboxc, 1 , wx.ALIGN_TOP)
        lastbox.Add(btn1, 0 ,wx.ALIGN_CENTRE |wx.ALL, 10)
        lastbox.Add(btn2, 0 ,wx.ALIGN_CENTRE |wx.ALL, 10)

        self.textFile = wx.TextCtrl(panel, size=(140, -1), value='out.csv')
        lastbox.Add(self.textFile, 0 ,wx.ALIGN_LEFT |wx.ALL, 10)

        panel.SetSizer(lastbox)
        lastbox.Fit(panel)

        self.joint_commandsPub = rospy.Publisher("/gummi/joint_commands", JointState,  latch=True, queue_size=1)
        self.subscriber = rospy.Subscriber('/gummi/joint_states', JointState, self.cmdCallback)

        self.gui_available = True

    def onChecked(self, e):
        cb = e.GetEventObject()
        if cb.GetValue():
            for name in self.joints.keys():
                self.joints[name]['sldr_pos'].Enable(False)
                self.joints[name]['sldr_coc'].Enable(False)
            self.sendCommand(False)
        else:
            for name in self.joints.keys():
                self.joints[name]['sldr_pos'].Enable(True)
                self.joints[name]['sldr_coc'].Enable(True)

            self.sendCommand(True)

    def OnRefresh(self, evt):
        self.refreshed = True

    def OnSave(self, evt):
       fileName = self.textFile.GetValue()
       with open(fileName, 'wb') as csvfile:
           writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
           writer.writerow([self.joints[name]['sldr_pos'].GetValue()*(pi/180.0) for name in self.joints.keys()])

    def OnSliderScroll(self, evt) :
        self.sendCommand(True)

    def sendCommand(self, active):
        msg = JointState()

        if self.received:
            if active:
                sign = 1
            else:
                sign = -1

            with self.gui_lock:
                msg.position = [self.joints[name]['sldr_pos'].GetValue()*(pi/180.0) for name in self.joints.keys()]
                msg.name = self.joints.keys()
                msg.effort = [self.joints[name]['sldr_coc'].GetValue()/100.0 for name in self.joints.keys()]
                msg.velocity = [0]*len(self.joints.keys())

            msg.header.stamp = rospy.Time.now()
            self.joint_commandsPub.publish(msg)
            self.cmdRate.sleep()


    def cmdCallback(self, msg):
        if self.gui_available and self.refreshed:
            for name,pos,eff in zip(msg.name, msg.position, msg.effort):
                with self.gui_lock:
                    self.joints[name]['sldr_pos'].SetValue(int(pos/(pi/180.0)))
                    self.joints[name]['sldr_coc'].SetValue(int(100*eff))

            self.statesRate.sleep()
            self.refreshed = False

        if not self.received:
            for name in self.joints.keys():
                self.joints[name]['sldr_pos'].Enable(True)
                self.joints[name]['sldr_coc'].Enable(True)

            self.received = True
            self.refreshed = False


class MyApp(wx.App):
    def OnInit(self):
        frame = MyFrame(None, -1, 'slider.py')
        frame.Show(True)
        frame.Centre()

	# This was necessary because the wx object was killed, but ROS would try to call cmdCallback
	def on_end_session(e):
	    rospy.signal_shutdown("wx window closed")
	    e.Skip()

	frame.Bind(wx.EVT_END_SESSION, on_end_session)

	frame.Bind(wx.EVT_CLOSE, on_end_session)

        #while not rospy.is_shutdown():
        #    frame.r.sleep()

        return True

app = MyApp(0)
app.MainLoop()
