import React from "react";
import { NavLink } from "react-router-dom";

export default function Sidebar() {
  const nav = [
    { to: "/overview", label: "Overview" },
    { to: "/waveforms", label: "Waveforms" },
    { to: "/pq-events", label: "PQ Events" },
    { to: "/analytics", label: "Analytics" },
    { to: "/reports", label: "Reports" },
  ];

  return (
    <aside className="sidebar">
      <div className="brand">
        <div className="brandTitle">ILM Smart Plug</div>
        <div className="brandSub">Interactive Dashboard</div>
      </div>

      <nav className="nav">
        {nav.map((n) => (
          <NavLink key={n.to} to={n.to} className="navItem">
            {n.label}
          </NavLink>
        ))}
      </nav>
    </aside>
  );
}
