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
from ros_service_examples import srv as srv_module


def _resolve_mp3_inventory_service_type():
  candidates = []
  for name in dir(srv_module):
    if name.startswith('_'):
      continue
    obj = getattr(srv_module, name)
    if isinstance(obj, type) and hasattr(obj, 'Request') and hasattr(obj, 'Response'):
      candidates.append((name, obj))

  if not candidates:
    raise RuntimeError('No ROS2 service type found in ros_service_examples.srv')

  preferred = [c for c in candidates if ('mp3' in c[0].lower() and 'inventory' in c[0].lower())]
  if preferred:
    preferred.sort(key=lambda x: x[0])
    return preferred[0][1]

  candidates.sort(key=lambda x: x[0])
  return candidates[0][1]


def _log_message_fields(logger, prefix, msg):
  fields = msg.get_fields_and_field_types()
  logger.info(prefix)
  for field_name in fields:
    logger.info(f'  {field_name}: {getattr(msg, field_name)}')


def mp3_inventory_client():
  rclpy.init(args=sys.argv)
  node = rclpy.create_node('mp3_inventory_client')

  try:
    service_type = _resolve_mp3_inventory_service_type()
    client = node.create_client(service_type, 'mp3_inventory_interaction')

    while not client.wait_for_service(timeout_sec=1.0):
      node.get_logger().info('Waiting for service "mp3_inventory_interaction"...')

    # TODO:
    # - Initialize a ROS2 service client for the MP3 inventory service
    # - Query the service for inventory information using the provided interface
    # - Use the service responses to determine subsequent requests and control flow
    # - Log or print the inventory information obtained from the service

    pending_requests = []

    first_req = service_type.Request()
    if hasattr(first_req, 'command'):
      setattr(first_req, 'command', 'inventory')
    elif hasattr(first_req, 'query'):
      setattr(first_req, 'query', 'inventory')
    elif hasattr(first_req, 'mode'):
      setattr(first_req, 'mode', 'inventory')
    pending_requests.append(first_req)

    iterations = 0
    max_iterations = 100

    while pending_requests and iterations < max_iterations:
      req = pending_requests.pop(0)
      iterations += 1

      _log_message_fields(node.get_logger(), 'Sending request:', req)
      future = client.call_async(req)
      rclpy.spin_until_future_complete(node, future)

      if future.exception() is not None:
        raise future.exception()

      resp = future.result()
      _log_message_fields(node.get_logger(), 'Received response:', resp)

      # Response-driven follow-up requests (best-effort based on common field names)
      total = None
      for total_field in ('total_items', 'count', 'num_items', 'inventory_size'):
        if hasattr(resp, total_field):
          try:
            total = int(getattr(resp, total_field))
            break
          except Exception:
            pass

      if total is not None and total > 0:
        request_fields = service_type.Request().get_fields_and_field_types().keys()
        index_field = None
        for candidate in ('index', 'item_index', 'id', 'item_id'):
          if candidate in request_fields:
            index_field = candidate
            break

        if index_field is not None:
          for i in range(total):
            follow_up = service_type.Request()
            setattr(follow_up, index_field, i)
            if hasattr(follow_up, 'command'):
              setattr(follow_up, 'command', 'item')
            pending_requests.append(follow_up)
          total = None  # prevent duplicate scheduling from list-based branch below

      for list_field in ('inventory', 'items', 'tracks', 'songs', 'entries'):
        if hasattr(resp, list_field):
          entries = getattr(resp, list_field)
          try:
            if len(entries) == 0:
              continue
          except Exception:
            continue

          request_fields = service_type.Request().get_fields_and_field_types().keys()
          key_field = None
          for candidate in ('name', 'title', 'track', 'song', 'item'):
            if candidate in request_fields:
              key_field = candidate
              break

          if key_field is not None:
            for entry in entries:
              follow_up = service_type.Request()
              setattr(follow_up, key_field, str(entry))
              if hasattr(follow_up, 'command'):
                setattr(follow_up, 'command', 'details')
              pending_requests.append(follow_up)
          break

    # END OF TODO

  except Exception as e:
    print(f"Service call failed: {e}")
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  mp3_inventory_client()