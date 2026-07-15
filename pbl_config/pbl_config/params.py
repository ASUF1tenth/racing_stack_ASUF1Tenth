import rclpy
from rcl_interfaces.srv import GetParameters


type_arr = ["not_set", "bool_value", "integer_value", "double_value", "string_value",
                         "byte_array_value", "bool_array_value", "integer_array_value",
                         "double_array_value", "string_array_value"]

def get_remote_parameter(node, remote_node_name, param_name, default=None):
        cli = node.create_client(GetParameters, remote_node_name + '/get_parameters')
        while not cli.wait_for_service(timeout_sec=1):
            node.get_logger().info('service not available, returning default value for parameter: %s' % param_name)
            if default is not None:
                return default
        req = GetParameters.Request()
        req.names = [param_name]
        future = cli.call_async(req)

        while rclpy.ok():
            rclpy.spin_once(node)
            if future.done():
                try:
                    res = future.result()
                    return getattr(res.values[0], type_arr[res.values[0].type])
                except Exception as e:
                    node.get_logger().warn('Service call failed %r' % (e,))
                    if default is not None:
                        return default
                    else:
                        raise e
