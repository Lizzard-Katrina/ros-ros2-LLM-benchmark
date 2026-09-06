#!/usr/bin/env python3

"""
MP3 Inventory Service Server — simulates a small MP3 database.
"""

import rclpy
from rclpy.node import Node
from task_003_mp3_db_service.srv import MP3InventoryService


class Mp3InventoryServer(Node):
    def __init__(self):
        super().__init__('mp3_inventory_server')

        # Simulated MP3 database
        self.database = {
            'Abbey Road': ['Come Together', 'Something', 'Here Comes The Sun'],
            'Thriller': ['Wanna Be Startin Somethin', 'Thriller', 'Beat It'],
            'Back in Black': ['Hells Bells', 'Back in Black', 'You Shook Me All Night Long'],
        }

        self.srv = self.create_service(
            MP3InventoryService,
            'mp3_inventory_interaction',
            self.handle_request
        )
        self.get_logger().info('MP3 Inventory Server is ready.')

    def handle_request(self, request, response):
        self.get_logger().info(
            'Received request: request_string=%s, album=%s' %
            (request.request_string, request.album)
        )

        if request.request_string == 'album_list':
            response.response_string = 'ok'
            response.list_strings = list(self.database.keys())
        elif request.request_string == 'title_list':
            album = request.album
            if album in self.database:
                response.response_string = 'ok'
                response.list_strings = self.database[album]
            else:
                response.response_string = 'album_not_found'
                response.list_strings = []
        else:
            response.response_string = 'unknown_request'
            response.list_strings = []

        return response


def main(args=None):
    rclpy.init(args=args)
    node = Mp3InventoryServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()