import rclpy
import subprocess
from rclpy.node import Node
from std_msgs.msg import UInt8
from std_srvs.srv import SetBool
from motor_interfaces.srv import MotorState
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

BYTE_CMD_TOPIC = '/byte_cmd_placeholder'

RETRY_PERIOD_S = 1.0   # how often to recheck an unavailable service


def build_setbool_request():
    req = SetBool.Request()
    req.data = True
    return req


def build_roll_request():
    req = SetBool.Request()
    req.data = True
    return req


# --- bit 0-5 -> init service. Each entry: (service_name, service_type, request_builder) ---
INIT_SERVICES = [
    ("/init_climb",          SetBool,    build_setbool_request),  # bit 0
    ("/init_roll",           SetBool,    build_roll_request),     # bit 1
    ("/init_elev_vert",      SetBool,    build_setbool_request),  # bit 2
    # ("/init_elev_horiz",     SetBool,    build_setbool_request),  # bit 3
    ("/init_gripper_pitch",  SetBool,    build_setbool_request),  # bit 4
    ("/init_gripper_catch",  SetBool,    build_setbool_request),  # bit 5
]
# bit 6 -> fires all 6 init services at once
ALL_INIT_BIT = 6

# bit 7 -> launches ak_60_up node via ros2 run if it's not already running
LAUNCH_BIT = 7
LAUNCH_PKG  = "ak_pkg"
LAUNCH_NODE = "ak_60_up"
# process-list match string used by pgrep to check if it's already running.
# matches against the full command line so 'ros2 run ak_pkg ak_60_up' is found.
LAUNCH_PGREP_PATTERN = f"{LAUNCH_PKG} {LAUNCH_NODE}"


class ByteCmdDispatcher(Node):

    def __init__(self):
        super().__init__('byte_cmd_dispatcher')

        self.get_logger().info("ByteCmdDispatcher initialized")

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.byte_sub = self.create_subscription(
            UInt8, '/activate_motors_data', self.byte_callback, qos_profile
        )

        # one client per init service (index matches INIT_SERVICES)
        self._clients = [
            self.create_client(srv_type, name) for (name, srv_type, _) in INIT_SERVICES
        ]

        self.last_bits = [0] * 8       # for rising-edge check, all 8 bits tracked
        self.pending    = [False] * len(INIT_SERVICES)  # call in-flight per service

        # retries unavailable services, fires calls whose service just came up
        self.timer = self.create_timer(RETRY_PERIOD_S, self.retry_pending_services)

        # service_index -> True while waiting for that service to become available
        self.queued_requests = {}

    def byte_callback(self, msg: UInt8):
        byte_val = msg.data

        for bit in range(8):
            bit_val = (byte_val >> bit) & 1
            rising_edge = (bit_val == 1 and self.last_bits[bit] == 0)
            self.last_bits[bit] = bit_val

            if not rising_edge:
                continue

            if bit == ALL_INIT_BIT:
                self.get_logger().info(f"bit {ALL_INIT_BIT} rising edge -> firing all init services")
                for i in range(len(INIT_SERVICES)):
                    self.call_init_service(i)
            elif bit == LAUNCH_BIT:
                self.get_logger().info(f"bit {LAUNCH_BIT} rising edge -> checking ak_60_up")
                self.launch_ak_60_up_if_not_running()
            elif bit < len(INIT_SERVICES):
                self.call_init_service(bit)

    def is_ak_60_up_running(self):
        # pgrep -f matches against the full command line, so this finds
        # 'ros2 run ak_pkg ak_60_up' specifically, not just any process
        # named ak_60_up.
        result = subprocess.run(
            ["pgrep", "-f", LAUNCH_PGREP_PATTERN],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        return result.returncode == 0   # 0 = at least one match found

    def launch_ak_60_up_if_not_running(self):
        if self.is_ak_60_up_running():
            self.get_logger().info(f"'{LAUNCH_NODE}' already running, doing nothing")
            return

        self.get_logger().info(f"'{LAUNCH_NODE}' not running, launching now")
        # Popen (not run/call) so this doesn't block the node, and the
        # child keeps running independently of this dispatcher process.
        subprocess.Popen(
            ["ros2", "run", LAUNCH_PKG, LAUNCH_NODE],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True,
        )

    def call_init_service(self, i):
        if self.pending[i]:
            # already running for this service, don't double-call
            name = INIT_SERVICES[i][0]
            self.get_logger().info(f"'{name}' already pending, skip")
            return

        client = self._clients[i]

        if not client.service_is_ready():
            name = INIT_SERVICES[i][0]
            self.get_logger().warn(f"'{name}' not available, will retry")
            self.queued_requests[i] = True
            return

        self.send_request(i)

    def send_request(self, i):
        name, _, build_request = INIT_SERVICES[i]
        client = self._clients[i]
        req = build_request()

        self.pending[i] = True
        future = client.call_async(req)
        future.add_done_callback(lambda f, idx=i: self.service_response_callback(f, idx))

        self.get_logger().info(f"calling '{name}'")

    def service_response_callback(self, future, i):
        name = INIT_SERVICES[i][0]
        self.pending[i] = False
        try:
            result = future.result()
            self.get_logger().info(
                f"'{name}' -> success={result.success}, message={result.message}"
            )
        except Exception as e:
            self.get_logger().error(f"'{name}' call failed: {e}")

    def retry_pending_services(self):
        # checks any service that was requested but wasn't up yet
        for i in list(self.queued_requests.keys()):
            client = self._clients[i]
            if client.service_is_ready():
                del self.queued_requests[i]
                self.send_request(i)


def main(args=None):
    rclpy.init(args=args)
    node = ByteCmdDispatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()