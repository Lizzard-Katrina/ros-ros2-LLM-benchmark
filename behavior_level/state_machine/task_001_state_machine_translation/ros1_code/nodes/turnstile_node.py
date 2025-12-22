#!/usr/bin/env python
import rospy
from ros1_code.todo.state_locked import LockedState
from ros1_code.todo.state_unlocked import UnlockedState
from ros1_code.todo.state_transition import TurnstileSM

def main():
    rospy.init_node('turnstile_node')
    sm = TurnstileSM()   # 包含 state addition 和 transitions
    sm.run()             # execute SMACH state machine
    rospy.spin()

if __name__ == '__main__':
    main()
