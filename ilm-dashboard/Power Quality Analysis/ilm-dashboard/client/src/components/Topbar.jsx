import React, { useEffect, useState } from "react";
import { api } from "../api";
import PowerToggle from "./PowerToggle";

export default function Topbar({ devices, activeDevice, setActiveDevice }) {
  const [isOn, setIsOn] = useState(1);

  // load state when device changes
  useEffect(() => {
    if (!activeDevice) return;

    api.get(`/api/state/${activeDevice}`).then((res) => {
      setIsOn(res.data.is_on ? 0 : 1);
    });
  }, [activeDevice]);

  // toggle state
  const toggle = async () => {
  // UI state (what you see on dashboard)
  const uiState = isOn ? 0 : 1;
  setIsOn(uiState);

  // HARDWARE is inverted, so send opposite to backend
  const hardwareState = uiState ? 0 : 1;

  await api.post(`/api/state/${activeDevice}`, { is_on: hardwareState });
};


  return (
    <div className="topbar">
      <div className="devicePick">
        <span className="muted">Active Device</span>

        <select
          value={activeDevice}
          onChange={(e) => setActiveDevice(e.target.value)}
        >
          {devices.map((d) => (
            <option key={d.id} value={d.id}>
              {d.name}
            </option>
          ))}
        </select>

        <PowerToggle isOn={isOn === 1} onToggle={toggle} />
      </div>

      <div className="status">
        <span className="dot" />
        Connected
      </div>
    </div>
  );
}
