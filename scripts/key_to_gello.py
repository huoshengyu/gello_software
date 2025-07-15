#! /usr/bin/env python3

# General imports
import numpy as np
import rospy
import sensor_msgs.msg

# Keyboard input imports
from pynput import keyboard

import sys

gello = {
    # 'key':[i,j,k]
    # where i,j select input channel [axes, buttons][i][j]
    # k selects input value
    # Left stick:   x = -axes[0], y = axes[1]
    # Right stick:  x = -axes[3], y = axes[4]
    # Dpad:         L/R = -axes[6], U/D = axes[7]
    # Buttons:      [X, O, S, T, LB, RB, LT, RT, Share, Menu, Xbox, Lstick, Rstick]
    # Keyboard Input    Gamepad Input       Gello Output        
    'w':[0, 1, 1],      # Lstick up         Elbow tilt up
    's':[0, 1,-1],      # Lstick down       Elbow tilt down
    'a':[0, 0, 1],      # Lstick left       Forearm roll counterclockwise
    'd':[0, 0,-1],      # Lstick right      Forearm roll clockwise
    'i':[0, 4, 1],      # Rstick up         Wrist pitch up
    'k':[0, 4,-1],      # Rstick down       Wrist pitch down
    'j':[0, 3, 1],      # Rstick left       Wrist roll counterclockwise
    'l':[0, 3,-1],      # Rstick right      Wrist roll clockwise
    'q':[1, 6, 1],      # LT                Shoulder pan counterclockwise
    'o':[1, 7, 1],      # RT                Shoulder pan clockwise
    'e':[1, 4, 1],      # LB                Shoulder lift up
    'u':[1, 5, 1],      # RB                Shoulder lift down
    'f':[0, 6, 1],      # Dpad left                             
    'h':[0, 6,-1],      # Dpad right                            
    't':[0, 7, 1],      # Dpad up                               
    'g':[0, 7,-1],      # Dpad down                             
    '1':[1, 0, 1],      # X/Down            Close gripper       
    '2':[1, 1, 1],      # O/Right           Open gripper        
    '3':[1, 2, 1],      # S/Left                                
    '4':[1, 3, 1],      # T/Up                                  
    '0':[1, 8, 1],      # Share             Controller haptics  
    '-':[1, 9, 1],      # Menu              Home pose           
    '=':[1,10, 1],      # PS4               Frame change        
    'x':[1,11, 1],      # Lstick press      Prev waypoint       
    ',':[1,12, 1],      # Rstick press      Next waypoint       
}

class KeyToGello():

    def __init__(self):
        self.rate = rospy.Rate(60) # 60 Hz publishing loop rate
        self.gello_fake_pub = rospy.Publisher("gello/fake", sensor_msgs.msg.Joy, queue_size=5)
        self.joy = sensor_msgs.msg.Joy()

        # Set controller type
        self.keybinds = gello

        # Keep a record of what keys should be when not pressed
        self._default_axes = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]) # Triggers should be 1 when unpressed
        self._default_buttons = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0]) # Need 13 buttons to accomodate PS4 controller (Xbox needs 11)
        # Initialize joystick values
        self.joy.axes = np.copy(self._default_axes)
        self.joy.buttons = np.copy(self._default_buttons)

        # Create keyboard listener object
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()
    
    def on_press(self, key):
        # If key in binds, send command value (only takes one key at a time)
        # Note that instance is checked first to avoid requesting nonexistent attributes
        if isinstance(key, keyboard.KeyCode) and key.char in self.keybinds.keys():
            k = key.char
            if self.keybinds[k][0] == 0:
                self.joy.axes[self.keybinds[k][1]] = self.keybinds[k][2]
            elif self.keybinds[k][0] == 1:
                self.joy.buttons[self.keybinds[k][1]] = self.keybinds[k][2]
        elif isinstance(key, keyboard.Key) and key == keyboard.Key.esc: # If key==esc, exit
            sys.exit()
    
    def on_release(self, key):
        # If key in binds, reset to default value (only takes one key at a time)
        # Note that instance is checked first to avoid requesting nonexistent attributes
        if isinstance(key, keyboard.KeyCode) and key.char in self.keybinds.keys():
            k = key.char
            if self.keybinds[k][0] == 0:
                self.joy.axes[self.keybinds[k][1]] = self._default_axes[self.keybinds[k][1]]
            elif self.keybinds[k][0] == 1:
                self.joy.buttons[self.keybinds[k][1]] = self._default_buttons[self.keybinds[k][1]]
    
    def publish(self):
        self.gello_fake_pub.publish(self.joy)
    
    def start(self):
        self.listener.start()
        rospy.loginfo("Keyboard listener started")
    
    def stop(self):
        self.listener.stop()
        rospy.loginfo("Keyboard listener stopped")

if __name__=="__main__":
    rospy.init_node('key_to_gello')

    key_to_gello = KeyToGello()
    while not rospy.is_shutdown():
        key_to_gello.publish()
        key_to_gello.rate.sleep()
    key_to_gello.stop()
