import React, { useEffect, useState } from "react";
import { api } from "../api";
import { socket } from "../socket";

function Card({ title, value, unit }) {
  return (
    <div className="card">
      <div className="cardTitle">{title}</div>
      <div className="cardValue">
        {value ?? "--"} <span className="unit">{unit}</span>
      </div>
    </div>
  );
}

export default function Overview({ deviceId }) {
  const [m, setM] = useState(null);

  useEffect(() => {
    if (!deviceId) return;

    api.get(`/api/metrics/${deviceId}`).then((res) => setM(res.data));

    socket.emit("subscribe", deviceId);
    socket.on("metrics", (data) => setM(data));

    return () => {
      socket.off("metrics");
      socket.emit("unsubscribe", deviceId);
    };
  }, [deviceId]);

  return (
    <div>
      <h1>Overview</h1>

      <div className="grid">
        <Card title="Current RMS" value={m?.current_rms} unit="A" />
        <Card title="Voltage RMS" value={m?.voltage_rms} unit="V" />
        <Card title="Active Power" value={m?.active_power} unit="W" />
        <Card title="Frequency" value={m?.frequency} unit="Hz" />
        <Card title="Power Factor" value={m?.power_factor} unit="" />
      </div>
    </div>
  );
}
