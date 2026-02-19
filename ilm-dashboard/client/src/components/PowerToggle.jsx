import React from "react";

export default function PowerToggle({ isOn, onToggle }) {
  return (
    <button className={`toggleBtn ${isOn ? "on" : "off"}`} onClick={onToggle}>
      {isOn ? "ON" : "OFF"}
    </button>
  );
}
