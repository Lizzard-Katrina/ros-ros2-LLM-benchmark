class ParamClient(object):

    def __init__(self, node, remote_node_name, param_change_callback=None):

        self._node = node
        self._remote_node_name = remote_node_name
        self._get_params_client = self._node.create_client(
            GetParameters, f'{remote_node_name}/get_parameters'
        )
        self._set_params_client = self._node.create_client(
            SetParameters, f'{remote_node_name}/set_parameters'
        )
        self._list_params_client = self._node.create_client(
            ListParameters, f'{remote_node_name}/list_parameters'
        )
        self._describe_params_client = self._node.create_client(
            DescribeParameters, f'{remote_node_name}/describe_parameters'
        )
        self._param_events_subscription = self._node.create_subscription(
            ParameterEvent, '/parameter_events', self._on_parameter_event,
            qos_profile_parameter_events
        )
        self._param_change_callback = param_change_callback

    def _on_parameter_event(self, event):
        if event.node != self._remote_node_name:
            return
        if self._param_change_callback is not None:
            self._param_change_callback(
                [Parameter.from_parameter_msg(p) for p in event.new_parameters],
                [Parameter.from_parameter_msg(p) for p in event.changed_parameters],
                [Parameter.from_parameter_msg(p) for p in event.deleted_parameters]
            )

    def list_parameters(self):
        list_params_request = ListParameters.Request()
        list_params_response = self._call_service(self._list_params_client, list_params_request)
        return list_params_response.result.names

    def get_parameters(self, names):
        get_params_request = GetParameters.Request()
        get_params_request.names = names
        get_params_response = self._call_service(self._get_params_client, get_params_request)
        return [
            Parameter.from_parameter_msg(ParameterMsg(name=name, value=value))
            for name, value in zip(names, get_params_response.values)
        ]

    def describe_parameters(self, names):
        describe_params_request = DescribeParameters.Request()
        describe_params_request.names = names
        describe_params_response = self._call_service(self._describe_params_client,
                                                      describe_params_request)
        return describe_params_response.descriptors

    def set_parameters(self, parameters):
        set_params_request = SetParameters.Request()
        set_params_request.parameters = [p.to_parameter_msg() for p in parameters]
        return self._call_service(self._set_params_client, set_params_request)

    def close(self):
        self._node.destroy_subscription(self._param_events_subscription)
        self._node.destroy_client(self._describe_params_client)
        self._node.destroy_client(self._list_params_client)
        self._node.destroy_client(self._set_params_client)
        self._node.destroy_client(self._get_params_client)

    def _call_service(self, client, request, timeout=1.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise AsyncServiceCallFailed(hint='timed out waiting for service')

        future = client.call_async(request)
        done_event = Event()

        def _done_callback(fut):
            done_event.set()

        future.add_done_callback(_done_callback)

        if not done_event.wait(timeout):
            raise AsyncServiceCallFailed(hint='the target node may not be spinning')

        if future.done():
            try:
                return future.result()
            except Exception as e:
                raise AsyncServiceCallFailed(hint=str(e))
        else:
            raise AsyncServiceCallFailed(hint='the target node may not be spinning')