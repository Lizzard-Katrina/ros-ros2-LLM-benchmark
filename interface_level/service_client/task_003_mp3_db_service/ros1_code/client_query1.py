#!/usr/bin/env python
import rospy

# TODO: import correct srv
# from ???.srv import QueryMP3

def main():
    rospy.init_node('client_query1')

    # TODO: wait for service
    # rospy.wait_for_service('mp3_query')

    # TODO: create proxy
    # query = rospy.ServiceProxy('mp3_query', QueryMP3)

    # TODO: send query: e.g. search by artist
    # resp = query("artist", "Beatles")
    # print(resp)

if __name__ == '__main__':
    main()
