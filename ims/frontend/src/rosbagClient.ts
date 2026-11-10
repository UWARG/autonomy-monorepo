import type { Ros } from 'roslib';

export type RecordingStatus = {
  recording: boolean;
  bag_path: string | null;
  error: string | null;
};

export type RosbagState = {
  connected: boolean;
  status: RecordingStatus | null;
  error: string | null;
};

export interface RosbagClient {
  subscribe(listener: (state: RosbagState) => void): () => void;
  setRecording(active: boolean): Promise<void>;
}

type ServiceResponse = { success: boolean; message: string };
let nextClientId = 0;

function parseStatus(data: string): RecordingStatus {
  const status = JSON.parse(data) as RecordingStatus;
  if (!status || typeof status.recording !== 'boolean'
    || !(status.bag_path === null || typeof status.bag_path === 'string')
    || !(status.error === null || typeof status.error === 'string')) {
    throw new Error('Invalid recording status received');
  }
  return status;
}

export function createRosbagClient(ros: Ros): RosbagClient {
  const command = { name: '/ims/rosbag/set_recording', type: 'std_srvs/srv/SetBool' };
  const query = { name: '/ims/rosbag/get_status', type: 'std_srvs/srv/Trigger' };
  const topic = '/ims/rosbag/status';
  const subscriptionId = `ims_rosbag:${++nextClientId}`;
  let requestId = 0;
  const listeners = new Set<(state: RosbagState) => void>();
  const pending = new Set<(error: Error) => void>();
  let state: RosbagState = { connected: ros.isConnected, status: null, error: null };
  let generation = 0;
  let lastStatus = 0;
  let statusRevision = 0;
  let staleTimer: ReturnType<typeof setInterval> | undefined;

  const emit = (next: RosbagState) => {
    state = next;
    listeners.forEach((listener) => listener(state));
  };

  function request(service: { name: string; type: string }, data: object, timeout: number): Promise<ServiceResponse> {
    if (!ros.isConnected) return Promise.reject(new Error('ROS disconnected'));
    return new Promise((resolve, reject) => {
      const id = `${subscriptionId}:request:${++requestId}`;
      let settled = false;
      const finish = (error: Error | null, response?: ServiceResponse) => {
        if (settled) return;
        settled = true;
        clearTimeout(timer);
        ros.off(id, onResponse);
        pending.delete(cancel);
        if (error) reject(error);
        else resolve(response!);
      };
      const cancel = (error: Error) => finish(error);
      const onResponse = (message: { result?: boolean; values: ServiceResponse | string }) => {
        if (message.result === false) finish(new Error(String(message.values)));
        else if (typeof message.values !== 'object' || !message.values
          || typeof message.values.success !== 'boolean' || typeof message.values.message !== 'string') {
          finish(new Error('Invalid recording controller response'));
        } else finish(null, message.values);
      };
      const timer = setTimeout(() => cancel(new Error('Recording controller did not respond')), timeout);
      pending.add(cancel);
      ros.on(id, onResponse);
      try {
        // Own the response listener so timeout/disconnect can remove it. ROSLIB's
        // Service wrapper otherwise retains listeners for permanently lost replies.
        ros.callOnConnection({ op: 'call_service', id, service: service.name, type: service.type, args: data });
      } catch (error) {
        finish(error instanceof Error ? error : new Error(String(error)));
      }
    });
  }

  function receive(message: { data: string }) {
    try {
      const status = parseStatus(message.data);
      lastStatus = Date.now();
      statusRevision++;
      emit({ connected: ros.isConnected, status, error: null });
    } catch (error) {
      emit({ ...state, status: null, error: String(error) });
    }
  }

  async function readStatus() {
    const current = generation;
    const previousRevision = statusRevision;
    try {
      const response = await request(query, {}, 5000);
      if (current !== generation) return;
      if (previousRevision !== statusRevision) return;
      if (!response.success) throw new Error(response.message);
      receive({ data: response.message });
    } catch (error) {
      // A live status publication is sufficient even if the status query times out.
      if (current === generation && statusRevision === previousRevision) {
        emit({ ...state, status: null, error: String(error) });
      }
    }
  }

  function onConnection() {
    generation++;
    lastStatus = 0;
    emit({ connected: true, status: null, error: null });
    ros.on(topic, receive);
    ros.callOnConnection({ op: 'subscribe', id: subscriptionId, topic, type: 'std_msgs/msg/String' });
    void readStatus();
  }

  function onClose() {
    generation++;
    pending.forEach((cancel) => cancel(new Error('ROS disconnected')));
    ros.off(topic, receive);
    emit({ connected: false, status: null, error: null });
  }

  return {
    subscribe(listener) {
      listeners.add(listener);
      listener(state);
      if (listeners.size === 1) {
        ros.on('connection', onConnection);
        ros.on('close', onClose);
        staleTimer = setInterval(() => {
          if (state.status && Date.now() - lastStatus > 20000) {
            emit({ ...state, status: null, error: 'Recording controller is unavailable' });
          }
        }, 1000);
        if (ros.isConnected) onConnection();
      }
      return () => {
        listeners.delete(listener);
        if (listeners.size === 0) {
          generation++;
          ros.off('connection', onConnection);
          ros.off('close', onClose);
          ros.off(topic, receive);
          if (ros.isConnected) ros.callOnConnection({ op: 'unsubscribe', id: subscriptionId, topic });
          clearInterval(staleTimer);
          pending.forEach((cancel) => cancel(new Error('Recording widget closed')));
          state = { connected: ros.isConnected, status: null, error: null };
        }
      };
    },
    async setRecording(active) {
      if (!state.connected || !state.status) throw new Error('Recording controller is unavailable');
      const current = generation;
      const response = await request(command, { data: active }, 15000);
      if (current !== generation) throw new Error('ROS connection changed');
      await readStatus();
      if (current !== generation) throw new Error('ROS connection changed');
      if (!response.success) throw new Error(response.message);
    },
  };
}
