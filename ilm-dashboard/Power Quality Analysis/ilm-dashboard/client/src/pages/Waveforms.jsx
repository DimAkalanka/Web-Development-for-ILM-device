import React, { useEffect, useRef, useState } from "react";
import { socket } from "../socket";

function MiniPlot({ title, data, unit }) {
  // simple SVG plot
  const w = 820, h = 140, pad = 16;
  const max = Math.max(...data, 1);
  const min = Math.min(...data, 0);

  const points = data.map((v, i) => {
    const x = pad + (i * (w - pad * 2)) / Math.max(data.length - 1, 1);
    const y =
      pad + ((max - v) * (h - pad * 2)) / Math.max(max - min, 1e-6);
    return `${x},${y}`;
  });

  return (
    <div className="plotCard">
      <div className="plotTitle">{title}</div>
      <div className="plotValue">{data.at(-1) ?? "--"} {unit}</div>

      <svg width="100%" viewBox={`0 0 ${w} ${h}`}>
        <polyline
          fill="none"
          stroke="currentColor"
          strokeWidth="3"
          points={points.join(" ")}
        />
      </svg>
    </div>
  );
}

export default function Waveforms({ deviceId }) {
  const [curr, setCurr] = useState([]);
  const [volt, setVolt] = useState([]);
  const [pow, setPow] = useState([]);

  const maxLen = 120; // last 120 seconds
  const push = (setter, val) =>
    setter((prev) => [...prev, val].slice(-maxLen));

  useEffect(() => {
    if (!deviceId) return;

    socket.emit("subscribe", deviceId);

    const onMetrics = (m) => {
      push(setCurr, m.current_rms);
      push(setVolt, m.voltage_rms);
      push(setPow, m.active_power);
    };

    socket.on("metrics", onMetrics);

    return () => {
      socket.off("metrics", onMetrics);
      socket.emit("unsubscribe", deviceId);
    };
  }, [deviceId]);

  return (
    <div>
      <h1>Waveforms</h1>

      <div className="plots">
        <MiniPlot title="Current RMS (trend)" data={curr} unit="A" />
        <MiniPlot title="Voltage RMS (trend)" data={volt} unit="V" />
        <MiniPlot title="Power RMS (trend)" data={pow} unit="W" />
      </div>
    </div>
  );
}
