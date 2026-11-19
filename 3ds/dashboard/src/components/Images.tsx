import { useEffect, useRef, useState } from "react";
import { useSubscription, sampleStatus } from "../hooks";
import { parseImage } from "../images";
import { Panel, type MonitorProps } from "./Telemetry";
import { EmptyState, StatusLine } from "./Presentation";
export function ImagesPanel(props: MonitorProps) {
  const [selected, setSelected] = useState(
    () => new URLSearchParams(window.location.search).get("image") ?? "",
  );
  const [decodeError, setDecodeError] = useState(false);
  const [dimensions, setDimensions] = useState("");
  const topics = props.topics.filter((t) =>
    ["sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"].includes(
      t.type,
    ),
  );
  const topic = topics.find((t) => t.name === selected);
  const sample = useSubscription(
    props.transport,
    props.state,
    topic?.name ?? "",
    topic?.type ?? "",
    parseImage,
    500,
  );
  const canvas = useRef<HTMLCanvasElement>(null);
  useEffect(() => {
    setDecodeError(false);
    const frame = sample.value;
    // Keep compressed-image dimensions until the next load event. Clearing
    // every frame can race onLoad and leave the metadata permanently blank.
    if (!frame) setDimensions("");
    if (frame?.pixels && frame.width && frame.height && canvas.current) {
      canvas.current.width = frame.width;
      canvas.current.height = frame.height;
      canvas.current
        .getContext("2d")
        ?.putImageData(
          new ImageData(frame.pixels, frame.width, frame.height),
          0,
          0,
        );
    }
  }, [sample.value]);
  return (
    <Panel title="Image preview" id="images">
      <label>
        Image topic
        <select
          value={topic?.name ?? ""}
          onChange={(e) => setSelected(e.target.value)}
        >
          <option value="">Select an image topic</option>
          {topics.map((t) => (
            <option key={t.name} value={t.name}>
              {t.name}
            </option>
          ))}
        </select>
      </label>
      <StatusLine
        status={sampleStatus(
          sample,
          props.now,
          props.state === "connected",
          !!selected && !topic,
        )}
      >
        {sample.value &&
        !sample.error &&
        !decodeError &&
        props.state === "connected" &&
        sample.received &&
        props.now - sample.received <= 5000
          ? " · Receiving frames"
          : ""}
        · ≤2 frames/s
      </StatusLine>
      <div className={`image ${sample.value ? "has-frame" : "no-frame"}`}>
        {sample.value &&
          (sample.error ||
            decodeError ||
            props.state !== "connected" ||
            !sample.received ||
            props.now - sample.received > 5000) && (
            <span className="frame-history">
              Last received frame · not current
            </span>
          )}
        {sample.value?.src ? (
          <img
            src={sample.value.src}
            alt="Latest ROS camera frame"
            onError={() => setDecodeError(true)}
            onLoad={(e) =>
              setDimensions(
                `${e.currentTarget.naturalWidth} × ${e.currentTarget.naturalHeight}`,
              )
            }
          />
        ) : sample.value?.pixels ? (
          <canvas ref={canvas} aria-label="Latest ROS camera frame" />
        ) : (
          <EmptyState title="No signal">
            {topic
              ? "Waiting for a supported camera frame."
              : "Select an image topic to begin receiving frames."}
          </EmptyState>
        )}
      </div>
      <p className="camera-topic">
        <span className="eyebrow">Image source</span>
        <code>{selected || "No image topic selected"}</code>
        {topic && <span className="meta">{topic.type}</span>}
      </p>
      {decodeError && <p role="alert">Unable to decode compressed image</p>}
      <p className="meta source-note">
        {sample.value?.width
          ? `${sample.value.width} × ${sample.value.height}`
          : dimensions}{" "}
        · rgb8, bgr8, rgba8, bgra8, mono8, JPEG and PNG supported.
      </p>
    </Panel>
  );
}
