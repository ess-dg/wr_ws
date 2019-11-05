from fsm import fsm, fsm_trans
from welder_class import welder
from welder_fsm import *


laser_states = {"LASER_OFF" : "0",
	            "LASER_ON"  : "1"}


def next_is_blade():

	return     (robot.next_is_blade and robot.laser_sync)


def next_is_not_blade():

	return (not robot.next_is_blade and robot.laser_sync)


def send_turn_on():
	print("LASER_FSM: Sending ON Command")
	robot.laser_cmd  = True
	robot.laser_sync = False
	robot.laser_pub.publish('f')


def send_turn_off():
	print("LASER_FSM: Sending OFF Command")
	robot.laser_cmd  = False
	robot.laser_sync = False
	robot.laser_pub.publish('s')

laser_trans_tt = [fsm_trans(laser_states["LASER_OFF"],     next_is_blade,  laser_states["LASER_ON"],  send_turn_on),
			      fsm_trans( laser_states["LASER_ON"], next_is_not_blade, laser_states["LASER_OFF"], send_turn_off)]

def laser_fsm_new():
	return fsm(laser_trans_tt,laser_states["LASER_OFF"])
