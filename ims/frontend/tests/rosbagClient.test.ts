import ROSLIB from 'roslib';
import { afterEach, describe, expect, it, vi } from 'vitest';
import { createRosbagClient } from '../src/rosbagClient';
import type { RosbagState } from '../src/rosbagClient';

// Real ROSLIB serialization/listeners, with only the external rosbridge socket replaced.
function bridge() {
  const ros = new ROSLIB.Ros({});
  let recording = true;
  let subscribed = false;
  let answer = true;
  let commandError: string | null = null;
  let holdQuery = false;
  let heldReply: (() => void) | undefined;
  const messages: { op: string; id: string; service: string; args: { data: boolean } }[] = [];
  const status = () => ({ recording, bag_path: '/bags/session', error: null });
  Object.assign(ros, { socket: {
    send(encoded: string) {
      const message = JSON.parse(encoded);
      messages.push(message);
      if (message.op === 'subscribe') subscribed = true;
      if (message.op === 'unsubscribe') subscribed = false;
      if (message.op !== 'call_service' || !answer) return;
      const querySnapshot = JSON.stringify(status());
      queueMicrotask(() => {
        if (message.service === '/ims/rosbag/get_status') {
          const reply = () => ros.emit(message.id, { values: { success: true, message: querySnapshot } });
          if (holdQuery) heldReply = reply;
          else reply();
        } else if (message.service === '/ims/rosbag/set_recording') {
          if (!commandError) recording = message.args.data;
          ros.emit(message.id, { values: { success: !commandError, message: commandError ?? 'OK' } });
        }
      });
    },
  }, isConnected: true });
  return {
    ros, messages,
    setAnswer(value: boolean) { answer = value; },
    setCommandError(value: string) { commandError = value; },
    setRecording(value: boolean) { recording = value; },
    holdQuery() { holdQuery = true; },
    releaseQuery() { heldReply?.(); },
    publish() { if (subscribed) ros.emit('/ims/rosbag/status', { data: JSON.stringify(status()) }); },
    disconnect() { Object.assign(ros, { isConnected: false }); ros.emit('close'); },
    reconnect() { Object.assign(ros, { isConnected: true }); ros.emit('connection'); },
  };
}

afterEach(() => vi.useRealTimers());

describe('ROS recording client', () => {
  it('does not overwrite a newer topic publication with a delayed status query', async () => {
    const server = bridge();
    server.setRecording(false);
    server.holdQuery();
    const client = createRosbagClient(server.ros);
    let state: RosbagState | undefined;
    const dispose = client.subscribe((value) => { state = value; });
    try {
      await Promise.resolve();
      server.setRecording(true);
      server.publish();
      expect(state?.status?.recording).toBe(true);
      server.releaseQuery();
      await Promise.resolve();
      expect(state?.status?.recording).toBe(true);
    } finally { dispose(); }
  });

  it('removes lost response listeners when disconnect cancels a request', async () => {
    const server = bridge();
    const client = createRosbagClient(server.ros);
    let state: RosbagState | undefined;
    const dispose = client.subscribe((value) => { state = value; });
    try {
      await vi.waitFor(() => expect(state?.status).toBeTruthy());
      server.setAnswer(false);
      const command = client.setRecording(false);
      const rejected = expect(command).rejects.toThrow('ROS disconnected');
      server.disconnect();
      await rejected;
      const call = server.messages.findLast((message) => message.service === '/ims/rosbag/set_recording');
      expect(server.ros.listenerCount(call!.id)).toBe(0);
    } finally { dispose(); }
  });
  it('reads existing recording state and sends absolute start/stop commands', async () => {
    const server = bridge();
    const client = createRosbagClient(server.ros);
    let state: RosbagState | undefined;
    const dispose = client.subscribe((value) => { state = value; });
    try {
      await vi.waitFor(() => expect(state?.status?.recording).toBe(true));
      await client.setRecording(false);
      expect(state?.status?.recording).toBe(false);
      await client.setRecording(true);
      expect(state?.status?.recording).toBe(true);
      expect(server.messages.filter((message) => message.service === '/ims/rosbag/set_recording')
        .map((message) => message.args.data)).toEqual([false, true]);
    } finally { dispose(); }
  });

  it('rejects failed commands and retains actual backend state', async () => {
    const server = bridge();
    const client = createRosbagClient(server.ros);
    let state: RosbagState | undefined;
    const dispose = client.subscribe((value) => { state = value; });
    try {
      await vi.waitFor(() => expect(state?.status).toBeTruthy());
      server.setCommandError('Recorder cannot stop');
      await expect(client.setRecording(false)).rejects.toThrow('Recorder cannot stop');
      expect(state?.status?.recording).toBe(true);
    } finally { dispose(); }
  });

  it('clears disconnected state, rejects pending commands, and re-reads state after reconnect', async () => {
    const server = bridge();
    const client = createRosbagClient(server.ros);
    let state: RosbagState | undefined;
    const dispose = client.subscribe((value) => { state = value; });
    try {
      await vi.waitFor(() => expect(state?.status).toBeTruthy());
      server.setAnswer(false);
      const command = client.setRecording(false);
      const rejected = expect(command).rejects.toThrow('ROS disconnected');
      server.disconnect();
      await rejected;
      expect(state?.status).toBeNull();
      expect(state?.connected).toBe(false);
      server.setAnswer(true);
      server.reconnect();
      await vi.waitFor(() => expect(state?.status?.recording).toBe(true));
    } finally { dispose(); }
  });

  it('disables control when the backend stops reporting state', async () => {
    vi.useFakeTimers();
    const server = bridge();
    const client = createRosbagClient(server.ros);
    let state: RosbagState | undefined;
    const dispose = client.subscribe((value) => { state = value; });
    try {
      await vi.advanceTimersByTimeAsync(0);
      expect(state?.status).toBeTruthy();
      await vi.advanceTimersByTimeAsync(21000);
      expect(state?.status).toBeNull();
      await expect(client.setRecording(false)).rejects.toThrow('unavailable');
    } finally { dispose(); }
  });
});
