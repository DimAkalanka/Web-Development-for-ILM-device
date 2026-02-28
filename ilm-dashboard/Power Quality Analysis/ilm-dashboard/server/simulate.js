function rand(min, max) {
  return min + Math.random() * (max - min);
}

function nowIso() {
  return new Date().toISOString();
}

function maybeEvent() {
  const r = Math.random();
  if (r < 0.03) return "Voltage Sag";
  if (r < 0.05) return "Voltage Swell";
  if (r < 0.06) return "Osc Transient";
  if (r < 0.07) return "Harmonics";
  return null;
}

function severityFor(type) {
  if (!type) return null;
  if (type === "Osc Transient") return "High";
  if (type === "Harmonics") return "Medium";
  return "Medium";
}

module.exports = { rand, nowIso, maybeEvent, severityFor };
