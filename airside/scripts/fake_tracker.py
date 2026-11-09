# Fake LTE tracker for testing tracker_bridge without hardware.
# Publishes 5 min of 1 Hz fixes in the tracker's MQTT format to broker.emqx.io.
# Run on your machine (not in the container): python3 airside/scripts/fake_tracker.py
# Requires: pip install "paho-mqtt>=2.0"

import json, math, time
from datetime import datetime, timezone
import paho.mqtt.client as mqtt

SESSION = "test-fake-tracker"

c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
c.connect("broker.emqx.io", 1883); c.loop_start()
lat0, lon0 = 50.0970884, -110.7328077   # Medicine Hat field
for i in range(300):                      # 5 min at 1 Hz
    lat = lat0 + 0.0002 * math.sin(i / 30)
    lon = lon0 + 0.0003 * math.cos(i / 30)
    gnss_time = datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")
    payload = {
        "session_id": SESSION,
        "sequence": i + 1,          # must start at 1; 0 or below is treated as a test sample
        "fix": True,
        "gnss_time": gnss_time,
        "latitude": lat,
        "longitude": lon,
    }
    c.publish(f"warg/efs-lte-tracker/{SESSION}/location", json.dumps(payload), qos=1)
    print(payload)
    time.sleep(1)