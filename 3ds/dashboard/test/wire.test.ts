import { once } from "node:events";
import { describe, it, expect } from "vitest";
import { WebSocketServer } from "ws";
import { LiveTransport } from "../src/ros";
import { object, text } from "../src/models";

describe("real roslib WebSocket traffic against a local protocol fixture", () => {
  it("discovers, receives telemetry, unsubscribes and sends only read operations", async () => {
    const server = new WebSocketServer({ host: "127.0.0.1", port: 0 });
    const packets: Record<string, unknown>[] = [];
    let stop = () => {};
    try {
      await once(server, "listening");
      const address = server.address();
      if (typeof address === "string" || !address)
        throw Error("Missing fixture port");
      server.on("connection", (socket) =>
        socket.on("message", (bytes) => {
          const packet = object(JSON.parse(bytes.toString()));
          packets.push(packet);
          if (packet.op === "call_service")
            socket.send(
              JSON.stringify({
                op: "service_response",
                id: packet.id,
                result: true,
                values: {
                  topics: ["/fixture"],
                  types: ["std_msgs/msg/String"],
                },
              }),
            );
          if (packet.op === "subscribe")
            socket.send(
              JSON.stringify({
                op: "publish",
                topic: "/fixture",
                msg: { data: "read-only fixture" },
              }),
            );
        }),
      );
      const transport = new LiveTransport(`ws://127.0.0.1:${address.port}`);
      await new Promise<void>((resolve, reject) => {
        const timeout = setTimeout(
          () => reject(Error("Fixture connection timed out")),
          2000,
        );
        stop = transport.start((state) => {
          if (state === "connected") {
            clearTimeout(timeout);
            resolve();
          }
        });
      });
      expect(await transport.discover()).toEqual([
        { name: "/fixture", type: "std_msgs/msg/String" },
      ]);
      let unsubscribe = () => {};
      const message = await new Promise<string>((resolve, reject) => {
        const timeout = setTimeout(
          () => reject(Error("Fixture telemetry timed out")),
          2000,
        );
        unsubscribe = transport.subscribe(
          "/fixture",
          "std_msgs/msg/String",
          (value) => {
            clearTimeout(timeout);
            resolve(text(object(value).data));
          },
        );
      });
      expect(message).toBe("read-only fixture");
      unsubscribe();
      await new Promise<void>((resolve) => setTimeout(resolve, 30));
      expect(packets.map((packet) => packet.op)).toEqual([
        "call_service",
        "subscribe",
        "unsubscribe",
      ]);
      expect(packets[0].service).toBe("/rosapi/topics");
      expect(packets[1]).toMatchObject({ throttle_rate: 500, queue_length: 1 });
    } finally {
      stop();
      for (const client of server.clients) client.terminate();
      await new Promise<void>((resolve) => server.close(() => resolve()));
    }
  });
});
