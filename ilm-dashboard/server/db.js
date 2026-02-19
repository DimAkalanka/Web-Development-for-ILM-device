const Database = require("better-sqlite3");

const db = new Database("ilm.db");

// Tables
db.exec(`
CREATE TABLE IF NOT EXISTS devices (
  id TEXT PRIMARY KEY,
  name TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS latest_metrics (
  device_id TEXT PRIMARY KEY,
  voltage_rms REAL,
  current_rms REAL,
  active_power REAL,
  frequency REAL,
  power_factor REAL,
  updated_at TEXT
);

CREATE TABLE IF NOT EXISTS pq_events (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  device_id TEXT NOT NULL,
  event_type TEXT NOT NULL,
  occurred_at TEXT NOT NULL,
  frequency REAL,
  voltage_value REAL,
  severity TEXT
);

CREATE TABLE IF NOT EXISTS daily_reports (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  device_id TEXT NOT NULL,
  report_date TEXT NOT NULL,   -- YYYY-MM-DD
  total_energy_kwh REAL,
  avg_power_w REAL,
  event_count INTEGER,
  created_at TEXT
);

CREATE UNIQUE INDEX IF NOT EXISTS idx_daily_unique
ON daily_reports(device_id, report_date);

CREATE TABLE IF NOT EXISTS device_state (
  device_id TEXT PRIMARY KEY,
  is_on INTEGER NOT NULL DEFAULT 1,
  updated_at TEXT
);

CREATE TABLE IF NOT EXISTS master_metrics (
  id INTEGER PRIMARY KEY CHECK (id=1),
  voltage_rms REAL,
  frequency REAL,
  updated_at TEXT
);
`);



// Seed devices
const seedDevices = [
  { id: "plug_a", name: "Plug A" },
  { id: "plug_b", name: "Plug B" },
  { id: "plug_c", name: "Plug C" },
  { id: "plug_d", name: "Plug D" },
];

const ins = db.prepare("INSERT OR IGNORE INTO devices (id, name) VALUES (?, ?)");
for (const dev of seedDevices) ins.run(dev.id, dev.name);


// Seed device ON/OFF state
const seedState = db.prepare(
  "INSERT OR IGNORE INTO device_state (device_id, is_on, updated_at) VALUES (?, 1, ?)"
);

for (const dev of seedDevices) {
  seedState.run(dev.id, new Date().toISOString());
}

db.prepare("INSERT OR IGNORE INTO master_metrics (id, voltage_rms, frequency, updated_at) VALUES (1, 0, 0, ?)")
  .run(new Date().toISOString());

module.exports = db;

