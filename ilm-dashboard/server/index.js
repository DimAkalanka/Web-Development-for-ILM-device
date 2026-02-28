const express = require("express");
const cors = require("cors");
const http = require("http");
const { Server } = require("socket.io");
require("dotenv").config();


//const db = require("./db");
// const { rand, nowIso, maybeEvent, severityFor } = require("./simulate");
const pool = require("./db_pg");


const app = express();
app.use(cors());
app.use(express.json());

const server = http.createServer(app);
const io = new Server(server, { cors: { origin: "*" } });

// ---------- REST API ----------

// Devices list
app.get("/api/devices", async (req, res) => {
  const { rows } = await pool.query("SELECT id, name FROM devices ORDER BY id");
  res.json(rows);
});


// Latest metrics for one device
app.get("/api/metrics/:deviceId", async (req, res) => {
  const { deviceId } = req.params;

  const masterQ = await pool.query(
    "SELECT voltage_rms, frequency, updated_at FROM master_metrics WHERE id=1"
  );
  const plugQ = await pool.query(
    "SELECT device_id, current_rms, active_power, power_factor, updated_at FROM latest_metrics WHERE device_id=$1",
    [deviceId]
  );

  const master = masterQ.rows[0] || {};
  const plug = plugQ.rows[0] || {};

  res.json({
    device_id: deviceId,
    voltage_rms: master.voltage_rms ?? null,
    frequency: master.frequency ?? null,
    current_rms: plug.current_rms ?? null,
    active_power: plug.active_power ?? null,
    power_factor: plug.power_factor ?? null,
    updated_at: plug.updated_at ?? master.updated_at ?? null,
  });
});



// PQ events for one device (latest 50)
app.get("/api/events/:deviceId", (req, res) => {
  const { deviceId } = req.params;
  const rows = db
    .prepare(
      "SELECT * FROM pq_events WHERE device_id=? ORDER BY occurred_at DESC LIMIT 50"
    )
    .all(deviceId);
  res.json(rows);
});

// Reports list for one device
app.get("/api/reports/:deviceId", (req, res) => {
  const { deviceId } = req.params;
  const rows = db
    .prepare(
      "SELECT report_date, total_energy_kwh, avg_power_w, event_count FROM daily_reports WHERE device_id=? ORDER BY report_date DESC LIMIT 30"
    )
    .all(deviceId);
  res.json(rows);
});

// Download report as JSON (later you can make PDF)
app.get("/api/reports/:deviceId/:date/download", (req, res) => {
  const { deviceId, date } = req.params;
  const row = db
    .prepare(
      "SELECT * FROM daily_reports WHERE device_id=? AND report_date=?"
    )
    .get(deviceId, date);

  if (!row) return res.status(404).json({ error: "Report not found" });

  res.setHeader("Content-Disposition", `attachment; filename=${deviceId}_${date}.json`);
  res.json(row);
});

// ---------- Device ON/OFF State ----------

// Get current ON/OFF state of a device
app.get("/api/state/:deviceId", async (req, res) => {
  const { deviceId } = req.params;

  const q = await pool.query(
    "SELECT device_id, is_on, updated_at FROM device_state WHERE device_id=$1",
    [deviceId]
  );

  res.json(q.rows[0] || { device_id: deviceId, is_on: 1 });
});


// Update ON/OFF state of a device
app.post("/api/state/:deviceId", async (req, res) => {
  const { deviceId } = req.params;
  const { is_on } = req.body;

  if (is_on !== 0 && is_on !== 1) {
    return res.status(400).json({ error: "is_on must be 0 or 1" });
  }

  await pool.query(
    `
    INSERT INTO device_state(device_id, is_on, updated_at)
    VALUES ($1, $2, now())
    ON CONFLICT(device_id) DO UPDATE SET
      is_on=excluded.is_on,
      updated_at=excluded.updated_at
    `,
    [deviceId, is_on]
  );

  io.to(deviceId).emit("state", { device_id: deviceId, is_on });

  res.json({ device_id: deviceId, is_on });
});


// Master ESP32 sends Voltage RMS + Frequency
app.post("/api/ingest/master", async (req, res) => {
  const { voltage_rms, frequency } = req.body;

  if (typeof voltage_rms !== "number" || typeof frequency !== "number") {
    return res.status(400).json({ error: "voltage_rms and frequency must be numbers" });
  }

  await pool.query(
    `
    INSERT INTO master_metrics (id, voltage_rms, frequency, updated_at)
    VALUES (1, $1, $2, now())
    ON CONFLICT(id) DO UPDATE SET
      voltage_rms=excluded.voltage_rms,
      frequency=excluded.frequency,
      updated_at=excluded.updated_at
    `,
    [voltage_rms, frequency]
  );

  io.emit("master_metrics", { voltage_rms, frequency });
  res.json({ ok: true });
});


app.post("/api/ingest/plug", async (req, res) => {
  const { device_id, current_rms, active_power, power_factor } = req.body;

  if (!device_id) return res.status(400).json({ error: "device_id is required" });

  await pool.query(
    `
    INSERT INTO latest_metrics(device_id, current_rms, active_power, power_factor, updated_at)
    VALUES ($1, $2, $3, $4, now())
    ON CONFLICT(device_id) DO UPDATE SET
      current_rms=excluded.current_rms,
      active_power=excluded.active_power,
      power_factor=excluded.power_factor,
      updated_at=excluded.updated_at
    `,
    [device_id, current_rms, active_power, power_factor]
  );

  // emit "metrics" (so your UI updates live)
  io.to(device_id).emit("metrics", { device_id, current_rms, active_power, power_factor });

  res.json({ ok: true });
});


  res.json({ ok: true });
});




// ---------- SOCKET.IO (Live stream) ----------
io.on("connection", (socket) => {
  console.log("Client connected:", socket.id);

  socket.on("subscribe", (deviceId) => {
    socket.join(deviceId);
  });

  socket.on("unsubscribe", (deviceId) => {
    socket.leave(deviceId);
  });

  socket.on("disconnect", () => {
    console.log("Client disconnected:", socket.id);
  });
});

// ---------- Data generator loop (replace with ESP32 later) ----------
const upsertMetrics = db.prepare(`
INSERT INTO latest_metrics(device_id, voltage_rms, current_rms, active_power, frequency, power_factor, updated_at)
VALUES (@device_id, @voltage_rms, @current_rms, @active_power, @frequency, @power_factor, @updated_at)
ON CONFLICT(device_id) DO UPDATE SET
  voltage_rms=excluded.voltage_rms,
  current_rms=excluded.current_rms,
  active_power=excluded.active_power,
  frequency=excluded.frequency,
  power_factor=excluded.power_factor,
  updated_at=excluded.updated_at
`);

const insertEvent = db.prepare(`
INSERT INTO pq_events(device_id, event_type, occurred_at, frequency, voltage_value, severity)
VALUES (?, ?, ?, ?, ?, ?)
`);

const upsertReport = db.prepare(`
INSERT INTO daily_reports(device_id, report_date, total_energy_kwh, avg_power_w, event_count, created_at)
VALUES (?, ?, ?, ?, ?, ?)
ON CONFLICT(device_id, report_date) DO UPDATE SET
  total_energy_kwh=excluded.total_energy_kwh,
  avg_power_w=excluded.avg_power_w,
  event_count=excluded.event_count,
  created_at=excluded.created_at
`);

function today() {
  const d = new Date();
  return d.toISOString().slice(0, 10);
}

/*
setInterval(() => {
  const devices = db.prepare("SELECT id FROM devices").all();

  for (const dev of devices) {
    const voltage = rand(210, 240);
    const current = rand(0.2, 5.0);
    const pf = rand(0.7, 1.0);
    const power = voltage * current * pf;
    const freq = rand(49.8, 50.2);

    const metrics = {
      device_id: dev.id,
      voltage_rms: Number(voltage.toFixed(1)),
      current_rms: Number(current.toFixed(2)),
      active_power: Number(power.toFixed(0)),
      frequency: Number(freq.toFixed(2)),
      power_factor: Number(pf.toFixed(2)),
      updated_at: nowIso(),
    };

    upsertMetrics.run(metrics);

    // maybe insert event
    const ev = maybeEvent();
    if (ev) {
      const vVal = ev.includes("Sag") ? rand(160, 200) : rand(245, 280);
      const sev = severityFor(ev);
      insertEvent.run(dev.id, ev, nowIso(), metrics.frequency, Number(vVal.toFixed(0)), sev);
    }

    // simple daily report update (dummy)
    const eventCount = db.prepare(
      "SELECT COUNT(*) as c FROM pq_events WHERE device_id=? AND occurred_at >= ?"
    ).get(dev.id, today() + "T00:00:00.000Z").c;

    upsertReport.run(
      dev.id,
      today(),
      Number(rand(0.2, 8.0).toFixed(2)),
      Number(rand(50, 900).toFixed(0)),
      eventCount,
      nowIso()
    );

    // emit live to subscribers
    io.to(dev.id).emit("metrics", metrics);
  }
}, 1000);
*/

const PORT = 5000;
server.listen(PORT, () => console.log("Server running on port", PORT));
