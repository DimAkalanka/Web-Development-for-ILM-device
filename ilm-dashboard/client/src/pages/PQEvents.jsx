import React, { useEffect, useState } from "react";
import { api } from "../api";

export default function PQEvents({ deviceId }) {
  const [events, setEvents] = useState([]);

  useEffect(() => {
    if (!deviceId) return;
    api.get(`/api/events/${deviceId}`).then((res) => setEvents(res.data));
  }, [deviceId]);

  return (
    <div>
      <h1>PQ Events</h1>

      <div className="table">
        <div className="row head">
          <div>Event Type</div>
          <div>Occurred At</div>
          <div>Severity</div>
          <div>Frequency</div>
          <div>Voltage</div>
        </div>

        {events.map((e) => (
          <div className="row" key={e.id}>
            <div>{e.event_type}</div>
            <div>{new Date(e.occurred_at).toLocaleString()}</div>
            <div>{e.severity}</div>
            <div>{e.frequency} Hz</div>
            <div>{e.voltage_value} V</div>
          </div>
        ))}
      </div>
    </div>
  );
}
