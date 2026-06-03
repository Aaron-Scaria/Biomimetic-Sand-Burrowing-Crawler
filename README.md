# 🔬 Biomimetic Sand-Burrowing Crawler

An advanced bio-inspired robotic platform modeled after the sub-surface excavation and locomotion mechanisms of the mole crab (*Emerita analoga*). This project implements closed-loop motor synchronization using an ESP32 variant to drive coordinated leg strokes, allowing the platform to burrow efficiently into granular media (sand).

---

## 🚀 Key Features
* **Biomimetic Leg Kinematics:** Replicates the anisotropic force generation found in digging marine crustaceans.
* **Closed-Loop Speed Control:** Powered by dual DC geared motors equipped with quadrature encoders for real-time tracking.
* **PI Synchronization Loop:** Features a dedicated synchronization control scheme (`PI_controller.ino`) that locks the phase of both limbs to prevent erratic movement or stall conditions under high soil resistance.
* **Machine Learning Edge Pipeline:** Contains data partitions and scripts designed to process sensor feedback for subterranean terrain analysis.

---

## 📂 Repository Structure

```text
├── 3D_Models/      # CAD assets and hardware design files (STEP/STL format)
├── Code/           # Device firmware and microcontroller logic
│   └── PI_controller.ino  # Main control loop, encoder interrupts, and PI sync engine
├── Images/         # System diagrams and hardware snapshots
│   └── robot.jpeg  # High-resolution image of the physical prototype
└── ML_processing/  # Datasets, partitions, and analysis scripts for gait optimization
