import type { Transport } from "./ros";
export const mockTopics = [
  { name: "/dashboard/tree", type: "std_msgs/msg/String" },
  { name: "/dashboard/temperatures", type: "std_msgs/msg/String" },
  {
    name: "/preflight/diagnostics",
    type: "diagnostic_msgs/msg/DiagnosticArray",
  },
  { name: "/mavros/state", type: "mavros_msgs/msg/State" },
  { name: "/camera/image_raw", type: "sensor_msgs/msg/Image" },
];
export const mockMessage = (name: string): unknown => {
  if (name === "/dashboard/tree")
    return {
      data: JSON.stringify({
        version: 1,
        stamp: Date.now() / 1000,
        tip: "wait",
        nodes: [
          {
            id: "root",
            parent: null,
            name: "Root (fixture)",
            status: "RUNNING",
          },
          {
            id: "wait",
            parent: "root",
            name: "WaitForSetpoint",
            status: "RUNNING",
          },
        ],
      }),
    };
  if (name === "/dashboard/temperatures")
    return {
      data: JSON.stringify({
        version: 1,
        stamp: Date.now() / 1000,
        host: "MOCK companion",
        source: "Synthetic fixture",
        cpu: [{ sensor: "mock CPU", celsius: 48.5 }],
        gpu: [{ sensor: "mock GPU", celsius: 44 }],
      }),
    };
  if (name === "/preflight/diagnostics")
    return {
      header: { stamp: { sec: Math.floor(Date.now() / 1000), nanosec: 0 } },
      status: [
        {
          name: "Mock camera",
          level: 0,
          message: "Synthetic fixture only",
          values: [],
        },
        {
          name: "Mock auxiliary link",
          level: 1,
          message: "Example warning; not a real safety check",
          values: [],
        },
      ],
    };
  if (name === "/mavros/state")
    return {
      connected: true,
      armed: false,
      guided: false,
      mode: "MOCK / STABILIZE",
      system_status: 3,
    };
  return {
    width: 160,
    height: 100,
    step: 480,
    encoding: "rgb8",
    data: Array.from(
      { length: 48000 },
      (_, i) =>
        ((Math.floor(i / 3) % 160) + Math.floor(Date.now() / 1000) * 5) % 255,
    ),
  };
};
export const mockTransport: Transport = {
  start(update) {
    update("connected");
    return () => {};
  },
  async discover() {
    return mockTopics;
  },
  subscribe(name, _type, callback, throttle = 500) {
    callback(mockMessage(name));
    const timer = setInterval(
      () => callback(mockMessage(name)),
      Math.max(throttle, 1000),
    );
    return () => clearInterval(timer);
  },
};
