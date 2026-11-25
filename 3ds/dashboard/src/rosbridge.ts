import { object, text } from "./models";

// Limits count JavaScript string code units, not decoded image bytes. The
// normal 640x480 RGB8/base64 publication is ~1.23 million characters.
export const ROSBRIDGE_LIMITS = {
  maxMessageChars: 16 * 1024 * 1024,
  maxBufferedChars: 32 * 1024 * 1024,
  maxGroups: 8,
  maxParts: 256,
  retentionMs: 10_000,
};
interface FragmentGroup {
  total: number;
  created: number;
  chars: number;
  parts: Map<number, string>;
}

/** JSON decoder for the compression:none transport used by this dashboard.
 * Reassembled messages return to roslib's existing topic/service dispatcher.
 * This is rosbridge application fragmentation, not WebSocket frame splitting.
 */
export class RosbridgeDecoder {
  private groups = new Map<string, FragmentGroup>();
  private chars = 0;

  constructor(
    private report: (error: string) => void,
    private now: () => number = Date.now,
    private limits = ROSBRIDGE_LIMITS,
  ) {}

  get pendingGroups() {
    return this.groups.size;
  }
  get bufferedChars() {
    return this.chars;
  }

  reset() {
    this.groups.clear();
    this.chars = 0;
  }

  expire() {
    for (const [id, group] of this.groups) {
      if (this.now() - group.created >= this.limits.retentionMs) {
        this.drop(id);
        this.report(`Incomplete rosbridge fragment group expired (${id})`);
      }
    }
  }

  decode(raw: unknown): Record<string, unknown> | undefined {
    this.expire();
    let id: string | undefined;
    try {
      if (typeof raw !== "string")
        throw Error("Expected JSON text (compression:none)");
      if (raw.length > this.limits.maxMessageChars)
        throw Error("ROSbridge packet exceeds size limit");
      const message = this.envelope(raw);
      if (message.op !== "fragment") return message;
      id = text(message.id);
      if (!id || id.length > 256) throw Error("Invalid fragment ID");
      const { num, total, data } = message;
      if (
        typeof num !== "number" ||
        typeof total !== "number" ||
        !Number.isSafeInteger(num) ||
        !Number.isSafeInteger(total) ||
        total < 1 ||
        total > this.limits.maxParts ||
        num < 0 ||
        num >= total ||
        typeof data !== "string"
      )
        throw Error("Invalid fragment index, total or data");

      let group = this.groups.get(id);
      if (!group) {
        if (this.groups.size >= this.limits.maxGroups)
          throw Error("Too many incomplete fragment groups");
        group = { total, created: this.now(), chars: 0, parts: new Map() };
        this.groups.set(id, group);
      }
      if (group.total !== total) throw Error("Conflicting fragment totals");
      const previous = group.parts.get(num);
      if (previous !== undefined) {
        if (previous !== data) throw Error("Conflicting duplicate fragment");
        return; // Identical duplicate: do not count it twice or extend its lifetime.
      }
      if (
        group.chars + data.length > this.limits.maxMessageChars ||
        this.chars + data.length > this.limits.maxBufferedChars
      ) {
        throw Error("Fragment reassembly exceeds size limit");
      }
      group.parts.set(num, data);
      group.chars += data.length;
      this.chars += data.length;
      if (group.parts.size !== total) return;

      // Join only once, after all validated indexes are present. Groups from
      // different messages may be interleaved and fragments may arrive unordered.
      const parts = Array.from({ length: total }, (_, index) =>
        group.parts.get(index)!,
      );
      this.drop(id);
      const complete = this.envelope(parts.join(""));
      if (complete.op === "fragment")
        throw Error("Nested fragment messages are unsupported");
      return complete;
    } catch (error) {
      if (id !== undefined) this.drop(id);
      this.report(error instanceof Error ? error.message : String(error));
      return;
    }
  }

  private envelope(serialized: string) {
    const message = object(JSON.parse(serialized));
    text(message.op);
    // The library dispatches these fields directly as event names. Validate
    // them before dispatch so malformed envelopes cannot throw in roslib.
    if (message.op === "publish") {
      text(message.topic);
      object(message.msg);
    } else if (message.op === "service_response") {
      text(message.id);
    } else if (message.op === "call_service") {
      text(message.service);
    }
    if (message.op === "png")
      throw Error("Unexpected rosbridge PNG envelope; use compression:none");
    return message;
  }

  private drop(id: string) {
    const group = this.groups.get(id);
    if (group) this.chars -= group.chars;
    this.groups.delete(id);
  }
}
