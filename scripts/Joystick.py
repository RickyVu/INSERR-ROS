#!/usr/bin/env python3
import rospy
import pubsub.json_message

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

# TODO REDESIGN
# - I think the entire joystick node requires a redesign
# - This node should just accept a unified raw input -> 
# handle on off switch states of buttons + axis preprocessing(if needed)
# then publish the on off states and axis positions ONLY
# - Needs to be more extensible and decoupled in terms of tools
# in other words should not be mentioning EM or gripper etc in this node
# example: just publish rctr-left (X-button on xbox controller) is ON!
# then the module in charge of a tool subscribed to this rctr-left
# will know its supposed to turn on the tool itself
# - So the workflow should be like this:
# Raw input node: Xbox node or PS4 node or XYZ Controller node doesn't matter
# publish control commands that should have the same structure and mapping
# This Joystick node receives it 
# and only publish which buttons are changed from on to off and vice versa, 
# also publish axis value accounting for deadzone etc
# Active Tool node subscribe to on/off of a button to turn on/off the tool
# The states of the tools (i.e EM state) should be handled within the tool
class Joystick:

    def __init__(self):
        super().__init__()

        self.pub_movement = pubsub.json_message.Publisher('inserr/input/movement', queue_size=10)
        self.pub_profile = pubsub.json_message.Publisher('inserr/input/profile', queue_size=10)
        pubsub.json_message.Subscriber('inserr/input/joystick_raw', self.joystick_raw_listener)
   

        # TODO Isolate these from joystick
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
        #pub.sendMessage("gamepad.selected_tool", message = {"gamepad_tool_index": tool_index}) # For GUI
        #pub.sendMessage("gamepad.em_states", message = self.em_states)
        self.bumper_hold_on = self.bumper_hold[tool_index]
        self.bumper_hold_default_sent = False

    '''
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
    '''


    def joystick_raw_listener(self, message):
        
        # GET JOYSTICK
        #for i in range(self.joystick.get_numaxes()):
        #    self.direct_input[i] = self.joystick.get_axis(i)
        self.direct_input = message["axes"]


        # GET BUTTONS
        self.y_input = [self.y_input[0], message["buttons"][0]]
        self.x_input = [self.x_input[0], message["buttons"][1]]
        self.b_input = [self.b_input[0], message["buttons"][2]]
        self.a_input = [self.a_input[0], message["buttons"][3]]
        self.lb_input = [self.lb_input[0], message["buttons"][4]]
        self.rb_input = [self.rb_input[0], message["buttons"][5]]
        self.back_input = [self.back_input[0], message["buttons"][6]]
        self.start_input = [self.start_input[0], message["buttons"][7]]
        self.l_stick_input = [self.l_stick_input[0], message["buttons"][8]]
        self.r_stick_input = [self.r_stick_input[0], message["buttons"][9]]
        # self.west_input = [self.west_input[0], hat_mapping(self.joystick.get_hat(0))[0]]
        # self.east_input = [self.east_input[0], hat_mapping(self.joystick.get_hat(0))[1]]
        # self.north_input = [self.north_input[0], hat_mapping(self.joystick.get_hat(0))[2]]
        # self.south_input = [self.south_input[0], hat_mapping(self.joystick.get_hat(0))[3]]
        
        
        self.north_input = [self.north_input[0], message["hat"][0]]
        self.west_input = [self.west_input[0], message["hat"][1]]
        self.east_input = [self.east_input[0], message["hat"][2]]
        self.south_input = [self.south_input[0], message["hat"][3]]

        # MAPPING IN WINDOWS
        #pub.sendMessage("gamepad.direct", message = {"gamepad_direct": self.direct_input}) # For GUI Tuple (LLR, LUD, RLR, RUD, BL, BR)
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
            self.pub_movement({"gamepad_movement": self.movement_message})

        if button_pressed(self.l_stick_input):
            self.thumb_profile_cycle = (self.thumb_profile_cycle-1)%len(ProfileChars)
            self.pub_profile({"gamepad_profile": ProfileChars[self.thumb_profile_cycle]})
        if button_pressed(self.r_stick_input):
            self.thumb_profile_cycle = (self.thumb_profile_cycle+1)%len(ProfileChars)
            self.pub_profile({"gamepad_profile": ProfileChars[self.thumb_profile_cycle]})

        #if button_pressed(self.x_input):
        #    self.control_invert = not self.control_invert
        #    pub.sendMessage("gamepad.invert", message = {"gamepad_invert": self.control_invert}) # For GUI

        #if button_pressed(self.a_input):
        #    self.a_counter += 1
        #    pub.sendMessage("gamepad.transect", message = {"gamepad_transect": int(self.a_counter % 2)})

        
        
        #if button_pressed(self.north_input):
        #    self.change_active_tool(0)

        #if button_pressed(self.west_input):
        #    self.change_active_tool(1)

        #if button_pressed(self.east_input):
        #    self.change_active_tool(2)

        #if button_pressed(self.south_input):
        #    self.change_active_tool(3)

        #if self.active_tool:
        #    self.tool_action()
            
        



if __name__ == '__main__':
    try:
        rospy.init_node('Joystick', anonymous=True)
        #param_dir = sys.argv[1]
        node_name = rospy.get_name()
        Joystick()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass