import React, { useEffect, useState } from "react";
import { Routes, Route, Navigate } from "react-router-dom";
import { api } from "./api";
import Sidebar from "./components/Sidebar";
import Topbar from "./components/Topbar";

import Overview from "./pages/Overview";
import Waveforms from "./pages/Waveforms";
import PQEvents from "./pages/PQEvents";
import Analytics from "./pages/Analytics";
import Reports from "./pages/Reports";

export default function App() {
  const [devices, setDevices] = useState([]);
  const [activeDevice, setActiveDevice] = useState("plug_a");

  useEffect(() => {
    api.get("/api/devices").then((res) => {
      setDevices(res.data);
      if (res.data?.[0]?.id) setActiveDevice(res.data[0].id);
    });
  }, []);

  return (
    <div className="layout">
      <Sidebar />
      <div className="main">
        <Topbar
          devices={devices}
          activeDevice={activeDevice}
          setActiveDevice={setActiveDevice}
        />

        <div className="page">
          <Routes>
            <Route path="/" element={<Navigate to="/overview" />} />
            <Route path="/overview" element={<Overview deviceId={activeDevice} />} />
            <Route path="/waveforms" element={<Waveforms deviceId={activeDevice} />} />
            <Route path="/pq-events" element={<PQEvents deviceId={activeDevice} />} />
            <Route path="/analytics" element={<Analytics deviceId={activeDevice} />} />
            <Route path="/reports" element={<Reports deviceId={activeDevice} />} />
          </Routes>
        </div>
      </div>
    </div>
  );
}
