# Robot Room Panel Control Interface

This repository contains the browser-based control interface for the **Robot Room** project, developed as part of ongoing research in the **Architectural Robotics Lab** at **Cornell University**.

Users can adjust the height of multiple robotic floor panels using an web interface that is responsive across various screen sizes. Commands are transmitted wirelessly via an **HC-05 Bluetooth module** to an **Arduino Mega**, which controls the physical linear actuators. The interface also visualizes the **target** and **actual** panel heights in real time.

---

## Demo
https://github.com/user-attachments/assets/dcf5c1c9-f327-4c27-bd4a-36ec7911e5e2

Simultate actual user flow with **Test Mode** without needing the Arduino or physical panels.

---

## Features

- Interactive panel height sliders
- Live elevation view
- Target vs actual panel visualization
- Wireless communication using HC-05 Bluetooth
- Built-in simulation mode (no hardware required)
- Safety validation for unsupported panel heights
- Responsive browser interface

---

# System Architecture

```
Browser (React UI)
        │
        │ Web Serial
        │
HC-05 Bluetooth Module
        │
Arduino Mega
        │
Motor Drivers
        │
Linear Actuators
        │
Position Sensors
```

The UI **never controls motors directly**.

Instead, it sends desired panel heights to the Arduino.

The Arduino is responsible for:

- actuator control
- synchronization
- sensor feedback
- safety logic
- motion planning

The Arduino continuously reports the actual measured panel heights back to the UI.

---

# Installation

Clone the repository

```bash
git clone https://github.com/<username>/<repository>.git
```

Install dependencies

```bash
npm install
```

Run the development server

```bash
npm run dev
```

Open

```
http://localhost:5173
```

---

# Connecting to the Physical Panels

## Hardware

Current hardware configuration

- Arduino Mega
- HC-05 Bluetooth Module
- Linear Actuators
- Dual Position Sensors
- Motor Drivers

---

## Pairing the HC-05

1. Pair the HC-05 in your operating system.
2. Open the web interface in **Google Chrome** or **Microsoft Edge**.
3. Click **Connect**.
4. Select the HC-05 serial device.

The interface communicates using the **Web Serial API**.

---

# Communication Protocol

## Browser → Arduino

Commands are sent as comma-separated values terminated by a newline.

Example

```
12.0,18.5,25.0
```

Meaning

```
Panel 1 = 12.0 in
Panel 2 = 18.5 in
Panel 3 = 25.0 in
```

---

## Arduino → Browser

The Arduino periodically returns the measured panel positions.

Example

```
POS,11.8,18.4,24.9
```

The UI ignores all other serial debug messages.
