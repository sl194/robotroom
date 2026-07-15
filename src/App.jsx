import React, { useState, useEffect, useRef, useCallback } from "react";
import { Plus, Minus, Bluetooth, BluetoothConnected, BluetoothSearching, RotateCcw } from "lucide-react";
import "./App.css";

/* -------------------------------------------------------------------------
   DATA LAYER
   Add a panel by adding one entry here. Everything else (sliders, the
   elevation view, validation) reads from this array.
------------------------------------------------------------------------- */

const MIN_IN = 0;
const MAX_IN = 40;
const STEP_IN = 0.5;
const MIN_WORKING_IN = 10; // hardware collapses below this unless fully stowed at 0

const PANELS = [
  { id: "p1", tag: "01", label: "Panel 1" },
  { id: "p2", tag: "02", label: "Panel 2" },
  { id: "p3", tag: "03", label: "Panel 3" },
];

// can be changed to fixed interval values to not be leading/prescriptive
const REFERENCE_LINES = [
  { value: 18, label: "seating" },
  { value: 29, label: "table" },
  { value: 40, label: "standing" },
]; 

const clamp = (v) => Math.min(MAX_IN, Math.max(MIN_IN, v));
const round1 = (v) => Math.round(v * 10) / 10;
const toCm = (inches) => round1(inches * 2.54);

// a panel is only unsafe when it's been raised off the floor but hasn't reached the hardware's minimum working height yet (i.e. 10in)
const isUnsafe = (value) => value > 0 && value < MIN_WORKING_IN;

/* -------------------------------------------------------------------------
   PANEL LINK
   HC-05 is Bluetooth Classic (serial), so a normal browser cannot reach it
   through Web Bluetooth/GATT. Pair the HC-05 in the operating system first,
   then select its serial port through the Web Serial API.

   A built-in simulation mode uses exactly the same target/actual data flow,
   so the UI can be tested without the Arduino or physical panels.
------------------------------------------------------------------------- */

const POSITION_LINE = /^(?:POS,)?(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?)$/;

function parsePositionLine(raw) {
  const match = raw.trim().match(POSITION_LINE);
  if (!match) return null;
  return { p1: Number(match[1]), p2: Number(match[2]), p3: Number(match[3]) };
}

function usePanelLink(onActualHeights) {
  const [status, setStatus] = useState("checking");
  // checking | unsupported | disconnected | connecting | connected | simulated
  const portRef = useRef(null);
  const readerRef = useRef(null);
  const writerRef = useRef(null);
  const keepReadingRef = useRef(false);
  const onActualRef = useRef(onActualHeights);
  onActualRef.current = onActualHeights;

  useEffect(() => {
    setStatus(typeof navigator !== "undefined" && "serial" in navigator ? "disconnected" : "unsupported");
  }, []);

  const readLoop = useCallback(async (port) => {
    const decoder = new TextDecoder();
    let buffer = "";
    keepReadingRef.current = true;

    try {
      while (keepReadingRef.current && port.readable) {
        const reader = port.readable.getReader();
        readerRef.current = reader;
        try {
          while (keepReadingRef.current) {
            const { value, done } = await reader.read();
            if (done) break;
            buffer += decoder.decode(value, { stream: true });

            const lines = buffer.split(/\r?\n/);
            buffer = lines.pop() ?? "";
            for (const line of lines) {
              const heights = parsePositionLine(line);
              if (heights) onActualRef.current(heights);
            }
          }
        } finally {
          reader.releaseLock();
          readerRef.current = null;
        }
      }
    } catch (error) {
      if (keepReadingRef.current) {
        console.error("Panel serial read failed", error);
        setStatus("disconnected");
      }
    }
  }, []);

  const connect = useCallback(async () => {
    if (!("serial" in navigator)) return;
    try {
      setStatus("connecting");
      const port = await navigator.serial.requestPort();
      await port.open({ baudRate: 9600 });
      portRef.current = port;
      writerRef.current = port.writable.getWriter();
      setStatus("connected");
      readLoop(port);
    } catch (error) {
      console.error("Could not open HC-05 serial port", error);
      setStatus("disconnected");
    }
  }, [readLoop]);

  const disconnect = useCallback(async () => {
    keepReadingRef.current = false;
    try {
      await readerRef.current?.cancel();
    } catch (_) {}
    readerRef.current = null;

    try {
      writerRef.current?.releaseLock();
    } catch (_) {}
    writerRef.current = null;

    try {
      await portRef.current?.close();
    } catch (_) {}
    portRef.current = null;
    setStatus("disconnected");
  }, []);

  const startSimulation = useCallback(() => {
    keepReadingRef.current = false;
    setStatus("simulated");
  }, []);

  const stopSimulation = useCallback(() => setStatus("disconnected"), []);

  const send = useCallback(
    async (heights) => {
      if (status === "simulated") return true;
      if (status !== "connected" || !writerRef.current) return false;

      try {
        // arduino reads commands only after a newline
        const line = `${PANELS.map((p) => (heights[p.id] ?? 0).toFixed(2)).join(",")}\n`;
        await writerRef.current.write(new TextEncoder().encode(line));
        return true;
      } catch (error) {
        console.error("Panel serial write failed", error);
        setStatus("disconnected");
        return false;
      }
    },
    [status]
  );

  useEffect(() => () => {
    keepReadingRef.current = false;
    try { readerRef.current?.cancel(); } catch (_) {}
    try { writerRef.current?.releaseLock(); } catch (_) {}
    try { portRef.current?.close(); } catch (_) {}
  }, []);

  return { status, connect, disconnect, startSimulation, stopSimulation, send };
}

/* -------------------------------------------------------------------------
   PRESENTATION COMPONENTS
   Visual styling for these class names lives in App.css.
------------------------------------------------------------------------- */

function StatusPill({ status, onConnect, onDisconnect, onStartSimulation, onStopSimulation }) {
  const map = {
    checking: { text: "Checking panels support", icon: BluetoothSearching, spin: true },
    unsupported: { text: "Web Serial unavailable — use simulation", icon: Bluetooth, spin: false },
    disconnected: { text: "Panels not connected", icon: Bluetooth, spin: false },
    connecting: { text: "Connecting to serial port", icon: BluetoothSearching, spin: true },
    connected: { text: "HC-05 serial connected", icon: BluetoothConnected, spin: false },
    simulated: { text: "Test mode — simulated panels", icon: BluetoothConnected, spin: false },
  };
  const m = map[status];
  const Icon = m.icon;
  const connected = status === "connected";
  const simulated = status === "simulated";

  return (
    <div className="status-wrap">
      <div className="status-pill" data-state={status}>
        <Icon size={14} className={m.spin ? "spin" : ""} />
        <span>{m.text}</span>
        {(status === "disconnected" || connected) && (
          <button className="status-action" onClick={connected ? onDisconnect : onConnect}>
            {connected ? "Disconnect" : "Connect"}
          </button>
        )}
      </div>
      <button className="simulation-btn" onClick={simulated ? onStopSimulation : onStartSimulation}>
        {simulated ? "Exit test mode" : "Test Mode"}
      </button>
    </div>
  );
}

function ElevationBars({ target, actual }) {
  return (
    <div className="elevation">
      {REFERENCE_LINES.map((ref) => (
        <div key={ref.label} className="ref-line" style={{ bottom: `${(ref.value / MAX_IN) * 100}%` }}>
          <span>{ref.label}</span>
        </div>
      ))}
      <div className="elevation-bars">
        {PANELS.map((panel) => {
          const targetVal = clamp(target[panel.id] ?? 0);
          const actualVal = clamp(actual[panel.id] ?? 0);
          const targetPct = (targetVal / MAX_IN) * 100;
          const actualPct = (actualVal / MAX_IN) * 100;
          const unsafe = isUnsafe(targetVal);
          return (
            <div className="bar-track" key={panel.id}>
              <div className="bar-value" data-unsafe={unsafe}>
                {round1(targetVal)}"
              </div>
              <div className="bar-target" data-unsafe={unsafe} style={{ height: `${targetPct}%` }} />
              <div className="bar-fill" data-unsafe={unsafe} style={{ height: `${actualPct}%` }} />
              <div className="bar-tag">{panel.tag}</div>
            </div>
          );
        })}
      </div>
  </div>
  );
}

const zeroHeights = () => {
  const h = {};
  PANELS.forEach((p) => (h[p.id] = 0));
  return h;
};

export default function App() {
  // target: what the sliders currently show (live while dragging)
  // confirmed: the last set of heights actually sent to the panels
  // actual: the panels' real, sensor-reported position
  const [target, setTarget] = useState(zeroHeights);
  const [confirmed, setConfirmed] = useState(zeroHeights);
  const [actual, setActual] = useState(zeroHeights);

  const handleActualHeights = useCallback((heights) => {
    setActual((prev) => ({ ...prev, ...heights }));
  }, []);

  const link = usePanelLink(handleActualHeights);

  const anyUnsafe = PANELS.some((p) => isUnsafe(target[p.id] ?? 0));
  const isDirty = PANELS.some((p) => Math.abs((target[p.id] ?? 0) - (confirmed[p.id] ?? 0)) > 0.05);
  const canConfirm = (link.status === "connected" || link.status === "simulated") && isDirty && !anyUnsafe;

  const setPanelHeight = (id, value) => {
    setTarget((prev) => ({ ...prev, [id]: clamp(round1(value)) }));
  };

  const step = (id, dir) => {
    setTarget((prev) => ({ ...prev, [id]: clamp(round1((prev[id] ?? 0) + dir * STEP_IN * 2)) }));
  };

  const resetAll = () => {
    setTarget(zeroHeights());
  };

  const confirmMove = async () => {
    if (!canConfirm) return;
    const accepted = await link.send(target);
    if (accepted) setConfirmed({ ...target });
  };

  // Hardware-free test rig: the outlined bars jump to the requested target; the solid bars then travel toward it at a believable actuator speed.
  useEffect(() => {
    if (link.status !== "simulated") return;

    const inchesPerSecond = 5;
    const tickMs = 50;
    const maxStep = inchesPerSecond * (tickMs / 1000);
    const timer = window.setInterval(() => {
      setActual((prev) => {
        let changed = false;
        const next = { ...prev };
        for (const panel of PANELS) {
          const current = prev[panel.id] ?? 0;
          const destination = confirmed[panel.id] ?? 0;
          const difference = destination - current;
          if (Math.abs(difference) <= maxStep) {
            next[panel.id] = destination;
          } else {
            next[panel.id] = round1(current + Math.sign(difference) * maxStep);
          }
          if (Math.abs(next[panel.id] - current) > 0.001) changed = true;
        }
        return changed ? next : prev;
      });
    }, tickMs);

    return () => window.clearInterval(timer);
  }, [link.status, confirmed]);

  return (
    <div className="app-shell">

      <div className="header">
        <div>
          <h1 className="header-title">Robot Room Panel Control</h1>
          <p className="header-sub">
            Adjust the height of each floor panel according to your preferences. Panels can rest flat at 0, or run anywhere from 10in
            upwards (heights &lt;10in are currently unsupported).
          </p>
        </div>
        <StatusPill
          status={link.status}
          onConnect={link.connect}
          onDisconnect={link.disconnect}
          onStartSimulation={link.startSimulation}
          onStopSimulation={link.stopSimulation}
        />
      </div>

      <div className="layout">
        <div className="schematic-panel">
          <div className="schematic-heading">
            <h2>Elevation</h2>
            <div className="legend">
              <span className="swatch outline" /> target
              <span className="swatch solid" /> actual
            </div>
          </div>
          <ElevationBars target={target} actual={actual} />
        </div>

        <div className="controls-panel">
          <div className="controls-heading">
            <h2>Panel heights</h2>
            <button className="reset-btn" onClick={resetAll}>
              <RotateCcw size={13} /> Reset to floor
            </button>
          </div>
          {PANELS.map((panel) => {
            const val = target[panel.id] ?? 0;
            const unsafe = isUnsafe(val);
            const remaining = round1(MIN_WORKING_IN - val);
            return (
              <div className="panel-row" key={panel.id}>
                <div className="panel-row-main">
                  <div className="panel-id">
                    <span className="tag">{panel.tag}</span>
                    {panel.label}
                  </div>
                  <input
                    type="range"
                    min={MIN_IN}
                    max={MAX_IN}
                    step={STEP_IN}
                    value={val}
                    data-unsafe={unsafe}
                    onChange={(e) => setPanelHeight(panel.id, parseFloat(e.target.value))}
                    aria-label={`${panel.label} height`}
                  />
                  <div className="stepper">
                    <button onClick={() => step(panel.id, -1)} aria-label={`Lower ${panel.label}`}>
                      <Minus size={13} />
                    </button>
                    <div className="readout" data-unsafe={unsafe}>
                      {round1(val).toFixed(1)} in <span className="cm">/ {toCm(val).toFixed(1)} cm</span>
                    </div>
                    <button onClick={() => step(panel.id, 1)} aria-label={`Raise ${panel.label}`}>
                      <Plus size={13} />
                    </button>
                  </div>
                </div>
                {unsafe && (
                  <div className="warning">
                    Minimum working height is {MIN_WORKING_IN} in. Raise {panel.label.toLowerCase()} by{" "}
                    {remaining.toFixed(1)} more in, or lower it back to 0.
                  </div>
                )}
              </div>
            );
          })}

          <button className="confirm-btn" onClick={confirmMove} disabled={!canConfirm}>
            {link.status !== "connected" && link.status !== "simulated"
              ? "Connect panels"
              : anyUnsafe
              ? "Resolve height warning to move"
              : isDirty
              ? "Confirm move"
              : "Panels at target"}
          </button>
        </div>
      </div>
    </div>
  );
}