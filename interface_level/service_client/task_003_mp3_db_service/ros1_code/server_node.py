#!/usr/bin/env python
import rospy

# TODO: import your custom service type
# from ???.srv import QueryMP3, QueryMP3Response

# TODO: define your MP3 database structure here

def handle_query(req):
    # TODO: process query_type and query_value
    # TODO: return QueryMP3Response with matching results
    pass

def main():
    rospy.init_node('mp3_db_server')

    # TODO: create the service server
    # service = rospy.Service('mp3_query', QueryMP3, handle_query)

    rospy.spin()

if __name__ == '__main__':
    main()
