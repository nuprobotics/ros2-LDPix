import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class TriggerProxy(Node):
    def __init__(self):
        super().__init__('trigger_proxy')

        # Required params + defaults (autotests)
        self.declare_parameter('service_name', '/trigger_service')
        self.declare_parameter('default_string', 'No service available')

        self._service_name = self.get_parameter('service_name').value
        self._default_string = self.get_parameter('default_string').value

        self._stored_string = self._default_string

        # Client to external service
        self._client = self.create_client(Trigger, '/spgc/trigger')

        # Server (name from parameter / launch)
        self._server = self.create_service(Trigger, self._service_name, self._on_service_call)

        self.get_logger().info(f"Providing service: {self._service_name}")
        self.get_logger().info("Trying to call /spgc/trigger once at startup...")

        # Call external trigger once (soon after startup)
        self._called_once = False
        self._timer = self.create_timer(0.2, self._try_call_once)

    def _try_call_once(self):
        if self._called_once:
            return
        self._called_once = True
        self._timer.cancel()

        # Check availability briefly
        if not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                f"/spgc/trigger not available. Using default_string: '{self._default_string}'"
            )
            self._stored_string = self._default_string
            return

        req = Trigger.Request()
        future = self._client.call_async(req)
        future.add_done_callback(self._on_trigger_response)

    def _on_trigger_response(self, future):
        try:
            resp = future.result()
            if resp is not None and resp.success:
                self._stored_string = resp.message
                self.get_logger().info(f"Stored string from /spgc/trigger: '{self._stored_string}'")
            else:
                # If response says not success, still store message if present; else default.
                self._stored_string = (resp.message if resp is not None and resp.message else self._default_string)
                self.get_logger().warn(f"/spgc/trigger returned unsuccessful. Stored: '{self._stored_string}'")
        except Exception as e:
            self._stored_string = self._default_string
            self.get_logger().warn(f"Failed calling /spgc/trigger: {e}. Using default_string.")

    def _on_service_call(self, request, response):
        response.success = True
        response.message = self._stored_string
        self.get_logger().info(f"Service called -> responding with: '{response.message}'")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TriggerProxy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
