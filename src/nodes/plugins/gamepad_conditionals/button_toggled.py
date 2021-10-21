#!/usr/bin/env python3
from ...nodes.conditional import Conditional

class ButtonToggled(Conditional):

    def __init__(self, button: int = 0):

        super().__init__()

        self.button = button
        self.toggle = False


    def condition(self, blackboard:dict):
        try:
            joy_button = blackboard['/joy'].buttons[self.button]

            if self.toggle:
                if joy_button == 1:
                    self.toggle = False
                    return False
                else:
                    return True
            else:
                if joy_button == 1:
                    self.toggle = True
                    return True
                else:
                    return False
            
        except:
            return False
