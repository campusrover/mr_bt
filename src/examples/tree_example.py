#!/usr/bin/env python3

import sys
sys.path.append("..")

import rospy
import time

from nodes import Selector, Sequencer, Action, Conditional


'''
This file is an example of how a behavior tree can be constructed in a python
script with the different types of nodes in the nodes.py file. This file shows a
simple behavior tree construction where we have a blackboard with "environment states"
and nodes which interact with and view that environment.
'''

class DoorOpen(Conditional):

    def condition(self, blackboard):
        
        open = blackboard['open']
        if open:
            print("The door is open!")
        else:
            print("The door is not opened")

        return open


class HasKey(Conditional):

    def condition(self, blackboard):
        
        has_key = blackboard['key']

        if has_key:
            print('I have a key')
        else:
            print("I don't have a key")

        return has_key


class PersonNearby(Conditional):

    def condition(self, blackboard):
        
        nearby = blackboard['person nearby']

        if nearby:
            print("There is a person nearby, I will ask for help")
        else:
            print("No one is around to help")

        return blackboard['person nearby']


class HasCrowbar(Conditional):

    def condition(self, blackboard):
        
        has_crowbar = blackboard['crowbar']

        if has_crowbar:
            print("I have a crowbar with me")
        else:
            print("I do not have a crowbar")

        return has_crowbar


class DoorThin(Conditional):

    def condition(self, blackboard):

        door_type = blackboard['door type']

        if door_type == 'thin':
            print("The door is thin enough to try to break down")
            return True
        else:
            print('The door is too thick to break down')
            return False


class OpenWithKey(Action):

    def tick(self, blackboard):

        if blackboard['door health'] > 10:
            print('Door is jammed with something, the key did not work')
            return 'failure'
        else:
            blackboard['open'] = True

            print('I just opened door with the key')

            return 'success'


class PersonOpen(Action):

    def tick(self, blackboard):

        if blackboard['person nice']:

            print('The person opened the door for me')
            blackboard['open'] = True
            return 'success'
        else:
            print('That person was nasty, they did not open the door')
            return 'failure'


class BreakDoor(Action):
    
    def tick(self, blackboard):

        blackboard['door health'] -= 1

        print('Hit door')

        if blackboard['door health'] > 0:
            return 'running'
        else:
            print('Door broken down')
            blackboard['open'] = True
            return 'success'


if __name__ == '__main__':

    blackboard = {
        'open':False,
        'key':True,
        'door jammed':True,
        'person nearby':True,
        'person nice':True,
        'crowbar':True,
        'door type':'thin',
        'door health':15
    }


    door_open = DoorOpen()
    has_key = HasKey()
    person_nearby = PersonNearby()
    has_crowbar = HasCrowbar()
    thin_door = DoorThin()

    open_door_key = OpenWithKey()
    person_open_door = PersonOpen()
    break_door = BreakDoor()

    key_seq = Sequencer([has_key, open_door_key])
    person_seq = Sequencer([person_nearby, person_open_door])
    break_seq = Sequencer([has_crowbar, thin_door, break_door])

    open_sel = Selector([door_open, key_seq, person_seq, break_seq])

    status = 'running'
    i = 1
    start = time.time()
    while status == 'running':
        print('\nTick {}:\n'.format(i))
        i += 1
        status = open_sel.tick(blackboard)
    print(time.time() - start)
    print(status)


    