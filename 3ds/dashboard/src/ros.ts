import ROSLIB from "roslib";
import { RosbridgeDecoder } from "./rosbridge";
import { object, array, text, type TopicInfo } from "./models";
export type Connection = "connecting" | "connected" | "disconnected";
export interface Transport {
  start: (update: (state: Connection, error?: string) => void) => () => void;
  discover: () => Promise<TopicInfo[]>;
  subscribe: (
    name: string,
    type: string,
    callback: (value: unknown) => void,
    throttle?: number,
  ) => () => void;
}
export const backoff = (attempt: number) =>
  Math.min(1000 * 2 ** Math.min(attempt, 5), 30000);
export class LiveTransport implements Transport {
  private ros: ROSLIB.Ros;
  private decoder = new RosbridgeDecoder((error) => {
    this.ros.emit("dashboard_protocol_error", error);
  });
  private sequence = 0;
  private pending = new Set<() => void>();
  private subscriptions = new Map<
    string,
    {
      type: string;
      id: string;
      listeners: Set<(value: unknown) => void>;
      receive: (value: unknown) => void;
    }
  >();
  constructor(private url: string) {
    const decoder = (raw: unknown, dispatch: (message: unknown) => void) => {
      const message = this.decoder.decode(raw);
      if (message) dispatch(message);
    };
    // roslib 1.4.1 supports transportOptions.decoder in SocketAdapter, but its
    // DefinitelyTyped declaration only lists the optional RTC fields.
    const transportOptions: RTCDataChannelInit & { decoder: typeof decoder } = {
      decoder,
    };
    this.ros = new ROSLIB.Ros({ groovyCompatibility: false, transportOptions });
  }
  start(update: (state: Connection, error?: string) => void) {
    let stopped = false,
      attempt = 0;
    let retry: ReturnType<typeof setTimeout> | undefined;
    const expiry = setInterval(() => this.decoder.expire(), 1000);
    const connect = () => {
      if (stopped) return;
      update("connecting");
      try {
        this.ros.connect(this.url);
      } catch (error) {
        closed(String(error));
      }
    };
    const closed = (error?: string) => {
      this.decoder.reset();
      for (const cancel of this.pending) cancel();
      if (stopped) return;
      update("disconnected", error);
      if (!retry)
        retry = setTimeout(() => {
          retry = undefined;
          connect();
        }, backoff(attempt++));
    };
    const handleClose = () => closed();
    const connected = () => {
      this.decoder.reset();
      attempt = 0;
      clearTimeout(retry);
      retry = undefined;
      update("connected");
    };
    const error = () =>
      closed("WebSocket error; check URL, network and rosbridge.");
    const protocolError = (message: string) =>
      update(
        this.ros.isConnected ? "connected" : "disconnected",
        `ROSbridge receive error: ${message}`,
      );
    this.ros.on("dashboard_protocol_error", protocolError);
    this.ros.on("connection", connected);
    this.ros.on("close", handleClose);
    this.ros.on("error", error);
    connect();
    return () => {
      stopped = true;
      clearTimeout(retry);
      clearInterval(expiry);
      this.decoder.reset();
      this.ros.off("dashboard_protocol_error", protocolError);
      this.ros.off("connection", connected);
      this.ros.off("close", handleClose);
      this.ros.off("error", error);
      for (const cancel of this.pending) cancel();
      this.ros.close();
    };
  }
  discover(): Promise<TopicInfo[]> {
    if (!this.ros.isConnected)
      return Promise.reject(Error("rosbridge disconnected"));
    return new Promise((resolve, reject) => {
      // roslib 1.x getTopics leaves its response listener after a timeout. Own
      // this single read-only rosapi request so every exit removes it.
      const id = `dashboard:topics:${++this.sequence}`;
      const cleanup = () => {
        clearTimeout(timer);
        this.ros.off(id, response);
        this.pending.delete(cancel);
      };
      const cancel = () => {
        cleanup();
        reject(Error("rosbridge disconnected"));
      };
      const response = (message: unknown) => {
        cleanup();
        try {
          const envelope = object(message);
          if (envelope.result === false)
            throw Error("rosapi rejected discovery");
          const value = object(envelope.values),
            names = array(value.topics),
            types = array(value.types);
          if (names.length !== types.length)
            throw Error("Malformed rosapi response");
          resolve(
            names
              .map((name, i) => ({ name: text(name), type: text(types[i]) }))
              .sort((a, b) => a.name.localeCompare(b.name)),
          );
        } catch (error) {
          reject(error);
        }
      };
      const timer = setTimeout(() => {
        cleanup();
        reject(Error("rosapi discovery timed out"));
      }, 5000);
      this.pending.add(cancel);
      this.ros.on(id, response);
      this.ros.callOnConnection({
        op: "call_service",
        id,
        service: "/rosapi/topics",
        type: "rosapi/srv/Topics",
        args: {},
        timeout: 5,
      });
    });
  }
  subscribe(
    name: string,
    type: string,
    callback: (value: unknown) => void,
    throttle = 500,
  ) {
    if (!this.ros.isConnected) return () => {};
    // Use roslib's message demultiplexer, with explicit lifecycle ownership.
    // Never queue subscriptions or unsubscriptions across connections.
    let subscription = this.subscriptions.get(name);
    if (!subscription) {
      const id = `dashboard:subscribe:${++this.sequence}`;
      const listeners = new Set<(value: unknown) => void>();
      const receive = (value: unknown) => {
        for (const listener of listeners) listener(value);
      };
      subscription = { type, id, listeners, receive };
      this.subscriptions.set(name, subscription);
      this.ros.on(name, receive);
      this.ros.callOnConnection({
        op: "subscribe",
        id,
        topic: name,
        type,
        throttle_rate: throttle,
        queue_length: 1,
        compression: "none",
      });
    }
    if (subscription.type !== type)
      throw Error("Conflicting topic message types");
    subscription.listeners.add(callback);
    return () => {
      subscription.listeners.delete(callback);
      if (subscription.listeners.size) return;
      this.subscriptions.delete(name);
      this.ros.off(name, subscription.receive);
      if (this.ros.isConnected)
        this.ros.callOnConnection({
          op: "unsubscribe",
          id: subscription.id,
          topic: name,
        });
    };
  }
}
