import { once } from "node:events";
import { expect, it, vi } from "vitest";
import { WebSocketServer } from "ws";
import { LiveTransport } from "../src/ros";
import { parseImage } from "../src/images";

it("delivers a fragmented RGB8 image before subsequent ordinary MAVROS telemetry", async () => {
  const server = new WebSocketServer({ host: "127.0.0.1", port: 0 });
  let stop = () => {};
  try {
    await once(server, "listening");
    const address = server.address();
    if (!address || typeof address === "string")
      throw Error("Missing fixture port");
    server.on("connection", (socket) =>
      socket.on("message", (raw) => {
        const request = JSON.parse(raw.toString());
        if (request.op !== "subscribe" || request.topic !== "/camera/image_raw")
          return;
        const serialized = JSON.stringify({
          op: "publish",
          topic: "/camera/image_raw",
          msg: { width: 1, height: 1, step: 3, encoding: "rgb8", data: "AAAA" },
        });
        const midpoint = Math.floor(serialized.length / 2);
        for (const [num, data] of [
          serialized.slice(0, midpoint),
          serialized.slice(midpoint),
        ].entries()) {
          socket.send(
            JSON.stringify({
              op: "fragment",
              id: "camera-frame",
              num,
              total: 2,
              data,
            }),
          );
        }
        socket.send(
          JSON.stringify({
            op: "publish",
            topic: "/mavros/state",
            msg: {
              connected: true,
              armed: false,
              guided: false,
              mode: "STABILIZE",
              system_status: 3,
            },
          }),
        );
      }),
    );
    const transport = new LiveTransport(`ws://127.0.0.1:${address.port}`);
    await new Promise<void>((resolve) => {
      stop = transport.start((state) => {
        if (state === "connected") resolve();
      });
    });
    const image = vi.fn(parseImage);
    const ordinary = new Promise<unknown>((resolve) =>
      transport.subscribe("/mavros/state", "mavros_msgs/msg/State", resolve),
    );
    transport.subscribe("/camera/image_raw", "sensor_msgs/msg/Image", image);
    expect(await ordinary).toMatchObject({
      connected: true,
      mode: "STABILIZE",
    });
    expect(image).toHaveBeenCalledOnce();
    expect([...image.mock.results[0].value.pixels!]).toEqual([0, 0, 0, 255]);
  } finally {
    stop();
    for (const socket of server.clients) socket.terminate();
    await new Promise<void>((resolve) => server.close(() => resolve()));
  }
});
