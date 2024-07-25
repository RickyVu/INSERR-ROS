import rospy
import pubsub.json_message

"""
# constants
DEADZONE_THRESHOLD_L = 0.1
DEADZONE_THRESHOLD_R = 0.1
DEADZONE_THRESHOLD_Z = 0.1
ProfileChars = ("A", "B", "C", "D")


def deadzoneleft(X):
    if X <(DEADZONE_THRESHOLD_L) and X > (-DEADZONE_THRESHOLD_L):
        return 0
    else:
        return X
def deadzoneright(X):
    if X <(DEADZONE_THRESHOLD_R) and X > (-DEADZONE_THRESHOLD_R):
        return 0
    else:
        return X

def deadzone_back(value):
    if value <(DEADZONE_THRESHOLD_Z) and value > (-DEADZONE_THRESHOLD_Z):
        return 0
    else:
        return value

def hat_mapping(hat_tuple):
    west = 0
    east = 0
    north = 0
    south = 0
    if hat_tuple[0]==1:
        east = 1
    if hat_tuple[0]==-1:
        west = 1
    if hat_tuple[1]==1:
        north = 1
    if hat_tuple[1]==-1:
        south = 1
    return west, east, north, south

def button_pressed(button_record):
    if button_record[0]!=button_record[1] and button_record[1]==1:
        button_record[0] = button_record[1]
        return True
    button_record[0] = button_record[1]
    return False

def button_hold(button_record):
    button_record[0] = button_record[1]
    if button_record[1] == 1:
        return True
    else:
        return False

def buttons_pressed(*button_records: "list[float, float]", all = False):
    if all:
        if False in [button_pressed(button_record) for button_record in button_records]:
            return False
        else:
            return True
    else:
        if True in [button_pressed(button_record) for button_record in button_records]:
            return True
        else:
            return False

class Joystick(Module):

    def __init__(self):
        super().__init__()


        # request from pygame services
        pygs = PyGameServices()
        self.joystick = pygs.get_joystick(0)

        if self.joystick is None:
            raise TypeError("Joystick not connected")

        #Active Tool Modes
        self.active_tool = ""
        self.active_tools = ("gamepad.gripper", "gamepad.EM1", "gamepad.EM2", "gamepad.actuator")
        self.bumper_hold = (True, False, False, True)  #Determines if the corresponding active tool requires holding down activation
        self.em_states ={"gamepad.EM1L": False, "gamepad.EM1R": False, "gamepad.EM2L": False, "gamepad.EM2R":False}
        self.last_tool = ""

        #Inputs
        self.a_input = [0, 0]
        self.b_input = [0, 0]
        self.x_input = [0, 0]
        self.y_input = [0, 0]
        self.lb_input = [0, 0]
        self.rb_input = [0, 0]
        self.back_input = [0, 0]
        self.start_input = [0, 0]
        self.guide_input = [0, 0]
        self.l_stick_input = [0, 0]
        self.r_stick_input = [0, 0]
        self.west_input = [0, 0]
        self.east_input = [0, 0]
        self.north_input = [0, 0]
        self.south_input = [0, 0]


        #State variables
        self.control_invert = False
        self.new_movement_message = [0, 0, 0, 0, 0, 0]
        self.movement_message = [0, 0, 0, 0, 0, 0]  #strafe, drive, yaw, updown, tilt, 0
        self.direct_input = [0, 0, 0, 0, 0, 0]
        self.bumper_hold_on = False
        self.bumper_hold_default_sent = False
        self.thumb_profile_cycle = 0
        self.a_counter = 0

    def em_message(self, tool_state):
        if tool_state==1:
            self.em_states[self.active_tool+"L"] = not self.em_states[self.active_tool+"L"]
        elif tool_state==-1:
            self.em_states[self.active_tool+"R"] = not self.em_states[self.active_tool+"R"]
        return {"L": self.em_states[self.active_tool+"L"], "R": self.em_states[self.active_tool+"R"]}

    def change_active_tool(self, tool_index : int):
        
        # Automatically handle change of active tool when index to self.active_tools is inputted
        # self.active_tools = [False, False, False, False] <= Gripper, EM_Left, EM_Right, Erector
        
        _new_tool = self.active_tools[tool_index]
        self.active_tool = _new_tool
        pub.sendMessage("gamepad.selected_tool", message = {"gamepad_tool_index": tool_index}) # For GUI
        pub.sendMessage("gamepad.em_states", message = self.em_states)
        self.bumper_hold_on = self.bumper_hold[tool_index]
        self.bumper_hold_default_sent = False

    def tool_action(self):
        if self.bumper_hold_on:
            if button_hold(self.lb_input):
                pub.sendMessage(self.active_tool, message = {"tool_state": 1})

                self.bumper_hold_default_sent = False
            elif button_hold(self.rb_input):
                pub.sendMessage(self.active_tool, message = {"tool_state": -1})
                self.bumper_hold_default_sent = False
            else:
                if not self.bumper_hold_default_sent:
                    pub.sendMessage(self.active_tool, message = {"tool_state": 0})
                    self.bumper_hold_default_sent = True
        else:
            if button_pressed(self.lb_input):
                pub.sendMessage(self.active_tool, message = self.em_message(1))
                pub.sendMessage("gamepad.em_states", message = self.em_states)
            if button_pressed(self.rb_input):
                pub.sendMessage(self.active_tool, message = self.em_message(-1))
                pub.sendMessage("gamepad.em_states", message = self.em_states)


    def run(self):
        
        # GET JOYSTICK
        for i in range(self.joystick.get_numaxes()):
            self.direct_input[i] = self.joystick.get_axis(i)

        # GET BUTTONS
        self.a_input = [self.a_input[0],self.joystick.get_button(0)]
        self.b_input = [self.b_input[0],self.joystick.get_button(1)]
        self.x_input = [self.x_input[0],self.joystick.get_button(2)]
        self.y_input = [self.y_input[0],self.joystick.get_button(3)]
        self.lb_input = [self.lb_input[0],self.joystick.get_button(4)]
        self.rb_input = [self.rb_input[0],self.joystick.get_button(5)]
        self.back_input = [self.back_input[0],self.joystick.get_button(6)]
        self.start_input = [self.start_input[0],self.joystick.get_button(7)]
        self.l_stick_input = [self.l_stick_input[0],self.joystick.get_button(8)]
        self.r_stick_input = [self.r_stick_input[0],self.joystick.get_button(9)]
        self.west_input = [self.west_input[0], hat_mapping(self.joystick.get_hat(0))[0]]
        self.east_input = [self.east_input[0], hat_mapping(self.joystick.get_hat(0))[1]]
        self.north_input = [self.north_input[0], hat_mapping(self.joystick.get_hat(0))[2]]
        self.south_input = [self.south_input[0], hat_mapping(self.joystick.get_hat(0))[3]]

        # MAPPING IN WINDOWS
        pub.sendMessage("gamepad.direct", message = {"gamepad_direct": self.direct_input}) # For GUI Tuple (LLR, LUD, RLR, RUD, BL, BR)
        LLR, LUD, RLR, RUD, BL, BR = self.direct_input
        LLR = 1*deadzoneleft(LLR)
        LUD = -1*deadzoneleft(LUD)
        RLR = 1*deadzoneright(RLR)
        RUD = -1*deadzoneright(RUD)

        BL = -((BL + 1) / 2)
        BR = (BR + 1) / 2
        BLR = deadzone_back(BL + BR)

        if self.control_invert:
            self.new_movement_message = [-LLR, -LUD, -RLR, -BLR, RUD, 0]
        else:
            self.new_movement_message = [LLR,  LUD, -RLR, -BLR,  -RUD, 0]    #(strafe, drive, yaw, updown, tilt / pitch, 0)

        ################### RLR / Yaw is flipped ##################


        # PUB LOOP
        if self.new_movement_message != self.movement_message:
            self.movement_message = self.new_movement_message[:]
            pub.sendMessage("gamepad.movement", message = {"gamepad_movement":self.movement_message})

        if button_pressed(self.l_stick_input):
            self.thumb_profile_cycle = (self.thumb_profile_cycle-1)%len(ProfileChars)
            pub.sendMessage("gamepad.profile", message = {"gamepad_profile": ProfileChars[self.thumb_profile_cycle]})
        if button_pressed(self.r_stick_input):
            self.thumb_profile_cycle = (self.thumb_profile_cycle+1)%len(ProfileChars)
            pub.sendMessage("gamepad.profile", message = {"gamepad_profile": ProfileChars[self.thumb_profile_cycle]})

        if button_pressed(self.x_input):
            self.control_invert = not self.control_invert
            pub.sendMessage("gamepad.invert", message = {"gamepad_invert": self.control_invert}) # For GUI

        if button_pressed(self.a_input):
            self.a_counter += 1
            pub.sendMessage("gamepad.transect", message = {"gamepad_transect": int(self.a_counter % 2)})

        if button_pressed(self.north_input):
            self.change_active_tool(0)

        if button_pressed(self.west_input):
            self.change_active_tool(1)

        if button_pressed(self.east_input):
            self.change_active_tool(2)

        if button_pressed(self.south_input):
            self.change_active_tool(3)

        if self.active_tool:
            self.tool_action()
            
        



if __name__ == '__main__':

    Joystick = Joystick()
    Joystick.start(60)

    pygs = PyGameServices()
    pygs.start(60)
"""

#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy

# constants
DEADZONE_THRESHOLD_L = 0.1
DEADZONE_THRESHOLD_R = 0.1
DEADZONE_THRESHOLD_Z = 0.1
ProfileChars = ("A", "B", "C", "D")

def deadzoneleft(X):
    if X <(DEADZONE_THRESHOLD_L) and X > (-DEADZONE_THRESHOLD_L):
        return 0
    else:
        return X
def deadzoneright(X):
    if X <(DEADZONE_THRESHOLD_R) and X > (-DEADZONE_THRESHOLD_R):
        return 0
    else:
        return X

def deadzone_back(value):
    if value <(DEADZONE_THRESHOLD_Z) and value > (-DEADZONE_THRESHOLD_Z):
        return 0
    else:
        return value

def joy_callback(data):
    # Left Stick
    left_stick_x = data.axes[0]  # Left Stick X-axis
    left_stick_y = data.axes[1]  # Left Stick Y-axis

    # Right Stick
    right_stick_x = data.axes[3]  # Right Stick X-axis
    right_stick_y = data.axes[4]  # Right Stick Y-axis

    # Hat Input
    hat_x = data.axes[6]  # Hat X-axis
    hat_y = data.axes[7]  # Hat Y-axis

    # Buttons
    LB = data.buttons[4]   # Left Bumper
    RB = data.buttons[5]   # Right Bumper
    LT = data.buttons[6]   # Left Trigger
    RT = data.buttons[7]   # Right Trigger
    A = data.buttons[0]    # A button
    B = data.buttons[1]    # B button
    X = data.buttons[2]    # X button
    Y = data.buttons[3]    # Y button
    
    rospy.loginfo("Left Stick: ({:.2f}, {:.2f}), Right Stick: ({:.2f}, {:.2f})".format(left_stick_x, left_stick_y, right_stick_x, right_stick_y))
    rospy.loginfo("Hat: ({:.2f}, {:.2f}), LB: {}, RB: {}, LT: {}, RT: {}, A: {}, B: {}, X: {}, Y: {}".format(hat_x, hat_y, LB, RB, LT, RT, A, B, X, Y))

def listener():
    rospy.init_node('joy_listener', anonymous=True)
    rospy.Subscriber("joy", Joy, joy_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()