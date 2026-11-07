"""ISOLATED TEST DATA ONLY: diagnostic transport and fake sensor files.

Never run this on the live drone/simulation graph. It supplies no real checks.
"""
import json
from pathlib import Path
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

root = Path('/tmp/monitor-validation')
sensors = root / 'FAKE_SENSOR_FILES'
for index, name, value in [(0, 'TEST_CPU-fixture', '48500'), (1, 'TEST_GPU-fixture', '51000')]:
    zone = sensors / 'class/thermal' / f'thermal_zone{index}'
    zone.mkdir(parents=True, exist_ok=True)
    (zone / 'type').write_text(name)
    (zone / 'temp').write_text(value)
control = root / 'control.json'
control.write_text(json.dumps({'level': 1, 'enabled': True}))
rclpy.init()
node = Node('ISOLATED_diagnostic_fixture')
publisher = node.create_publisher(DiagnosticArray, '/preflight/diagnostics', 1)


def publish():
    state = json.loads(control.read_text())
    if not state['enabled']:
        return
    message = DiagnosticArray()
    message.header.stamp = node.get_clock().now().to_msg()
    message.status = [DiagnosticStatus(
        name='TEST FIXTURE ONLY: example check', hardware_id='NOT_A_DRONE',
        level=bytes([state['level']]), message='Synthetic diagnostic transport test; not flight readiness',
        values=[KeyValue(key='test_data', value='true')],
    )]
    publisher.publish(message)


node.create_timer(0.5, publish)
try:
    rclpy.spin(node)
except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
