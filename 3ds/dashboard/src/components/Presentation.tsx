import type { ReactNode } from "react";
import { createPortal } from "react-dom";

// Presentation only: use the existing panel samples; never subscribe for summaries.
export function OverviewMetric({
  target,
  label,
  value,
}: {
  target?: HTMLElement | null;
  label: string;
  value: ReactNode;
}) {
  return target
    ? createPortal(
        <div className="overview-metric">
          <dt>{label}</dt>
          <dd>{value}</dd>
        </div>,
        target,
      )
    : null;
}

export function StatusLine({
  status,
  children,
}: {
  status: string;
  children?: ReactNode;
}) {
  const kind = status.includes("Invalid")
    ? "error"
    : status.includes("STALE")
      ? "stale"
      : status === "Disconnected"
        ? "disconnected"
        : status.includes("unavailable") || status.includes("Waiting")
          ? "waiting"
          : "receiving";
  return (
    <p className={`meta status-line ${kind}`}>
      <span className="status-mark" aria-hidden="true" />
      <span>
        {status}
        {children}
      </span>
    </p>
  );
}

export function EmptyState({
  title,
  children,
}: {
  title: string;
  children: ReactNode;
}) {
  return (
    <div className="empty-state">
      <span className="empty-mark" aria-hidden="true">
        —
      </span>
      <div>
        <strong>{title}</strong>
        <p>{children}</p>
      </div>
    </div>
  );
}
