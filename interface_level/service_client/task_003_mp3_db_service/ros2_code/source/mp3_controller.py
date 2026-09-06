#!/usr/bin/env python3

#  Software License Agreement (BSD License)
#
#  Copyright (c) 2015, Jan Winkler, Institute for Artificial Intelligence,
#  Universitaet Bremen.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Institute for Artificial Intelligence,
#     Universitaet Bremen, nor the names of its contributors may be
#     used to endorse or promote products derived from this software
#     without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

import sys
import rclpy
from rclpy.node import Node
from task_003_mp3_db_service.srv import MP3InventoryService


class Mp3InventoryClient(Node):
    def __init__(self):
        super().__init__('mp3_inventory_client')
        self.client = self.create_client(MP3InventoryService, 'mp3_inventory_interaction')

        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for mp3_inventory_interaction service...')

        self.query_inventory()

    def send_request(self, request_string, album):
        request = MP3InventoryService.Request()
        request.request_string = request_string
        request.album = album
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def query_inventory(self):
        try:
            response = self.send_request('album_list', '')

            print(' - Albums:')
            for album_item in response.list_strings:
                print('   * %s' % album_item)

                try:
                    response = self.send_request('title_list', album_item)

                    print('     Titles:')
                    for title_item in response.list_strings:
                        print('       o %s' % title_item)
                except Exception as e:
                    print('Service call failed: %s' % e)
        except Exception as e:
            print('Service call failed: %s' % e)


def main(args=None):
    rclpy.init(args=args)
    node = Mp3InventoryClient()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()