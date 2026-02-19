import React, { useEffect, useState } from "react";
import { api, API_BASE } from "../api";

export default function Reports({ deviceId }) {
  const [rows, setRows] = useState([]);

  useEffect(() => {
    if (!deviceId) return;
    api.get(`/api/reports/${deviceId}`).then((res) => setRows(res.data));
  }, [deviceId]);

  return (
    <div>
      <h1>Reports</h1>

      <div className="table">
        <div className="row head">
          <div>Date</div>
          <div>Total Energy</div>
          <div>Avg Power</div>
          <div>Events</div>
          <div>Download</div>
        </div>

        {rows.map((r) => (
          <div className="row" key={r.report_date}>
            <div>{r.report_date}</div>
            <div>{r.total_energy_kwh} kWh</div>
            <div>{r.avg_power_w} W</div>
            <div>{r.event_count}</div>
            <div>
              <a
                className="btn"
                href={`${API_BASE}/api/reports/${deviceId}/${r.report_date}/download`}
              >
                Download
              </a>
            </div>
          </div>
        ))}
      </div>
    </div>
  );
}
