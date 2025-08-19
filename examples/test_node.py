#!/usr/bin/env python3

import os
import asyncio
from pathlib import Path

import pycyphal
import pycyphal.application
from pycyphal.application import make_transport
from pycyphal.application.register import Natural16, Natural32, ValueProxy, Integer32, String

import uavcan.node
from uavcan.primitive.scalar import Integer32_1_0 as ScalarInt32
from voltbro.echo.echo_service_1_0 import echo_service_1_0

ECHO_PROVIDER_PORT = 248
DATA_PUBLISHER_PORT = 1808
DATA_SUBSCRIBER_PORT = 1811


class SimpleCyphalNode:
    def __init__(self, local_node_id: int, echo_target_id: int):
        self.local_node_id = local_node_id
        self.echo_target_id = echo_target_id

        node_info = uavcan.node.GetInfo_1.Response(
            software_version=uavcan.node.Version_1(major=1, minor=0),
            name="org.voltbro.cyphal_example_node",
        )
        self.node = pycyphal.application.make_node(
            node_info,
            Path(os.environ["VAR_DIR"]).resolve() / "cyphal.db",
            transport=make_transport(
                {
                    "uavcan.can.iface": ValueProxy("socketcan:vcan0"),
                    "uavcan.node.id": ValueProxy(Natural16([local_node_id])),
                    "uavcan.can.mtu": ValueProxy(Natural16([64])),
                    "uavcan.can.bitrate": ValueProxy(Natural32([1000000, 8000000])),
                }
            ),
        )
        self.node.heartbeat_publisher.mode = uavcan.node.Mode_1.OPERATIONAL
        self.node.heartbeat_publisher.vendor_specific_status_code = os.getpid() % 100

        # Setup registers (only as provider)
        self._setup_registers()

        # 4. Echo service provider
        self._setup_echo_provider()

        # 6. Data publisher
        self._setup_publisher()

        # 7. Data subscriber
        self._setup_subscriber()

    def _setup_registers(self):
        self.node.registry["example.str"] = String("example")
        self.node.registry["example.counter"] = Integer32([1234])

    def _setup_echo_provider(self):
        self.echo_srv = self.node.get_server(echo_service_1_0, ECHO_PROVIDER_PORT)

        async def handle_echo(req: echo_service_1_0.Request, metadata):
            print(f"[Provider] Received echo request: {req.ping}")
            return echo_service_1_0.Response(req.ping)

        print(f"[Provider] Started serving echo on port {ECHO_PROVIDER_PORT}")
        self.echo_srv.serve_in_background(handle_echo)

    def _setup_publisher(self):
        self.pub = self.node.make_publisher(ScalarInt32, DATA_PUBLISHER_PORT)

        async def publish_loop():
            value = 0
            while True:
                await self.pub.publish(ScalarInt32(value))
                print(f"[Publisher] Published {value}")
                value += 1
                await asyncio.sleep(2.0)

        asyncio.get_event_loop().create_task(publish_loop())

    def _setup_subscriber(self):
        self.sub = self.node.make_subscriber(ScalarInt32, DATA_SUBSCRIBER_PORT)

        async def on_message(msg: ScalarInt32, metadata):
            print(f"[Subscriber] Received {msg.value} from {metadata.source_node_id}")

        print(f"[Subscriber] Started receiving on port {DATA_SUBSCRIBER_PORT}")
        self.sub.receive_in_background(on_message)

    def start(self) -> None:
        self.node.start()

    def close(self) -> None:
        self.node.close()


async def main():
    local_id = int(sys.argv[1])
    target_id = int(sys.argv[2])

    node = SimpleCyphalNode(local_id, target_id)
    node.start()
    try:
        while True:
            await asyncio.sleep(1e-3)
    except KeyboardInterrupt:
        node.close()


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <local_node_id> <echo_target_id>")
        sys.exit(1)

    asyncio.get_event_loop().run_until_complete(main())
