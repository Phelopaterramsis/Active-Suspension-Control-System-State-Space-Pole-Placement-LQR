# 🚗 Active Suspension Control System – State-Space, Pole Placement & LQR

This folder contains my **Advanced Control Systems (Control 2)** project, where we modeled, analyzed, and controlled a **2-DOF active car suspension system** using state-space modeling, transfer functions, pole placement, and LQR. Simulations were performed using **MATLAB & Simulink**.

## 📌 Course

- Course: REE 409 – Control 2  
- University: University of Science and Technology – Zewail City  
- Supervisor: Dr. Mohamed Lotfi Eid Shaltout  
- Students: **Phelopater Ramsis**, Youssef Tarek  
- Semester: Fall 2025  
:contentReference[oaicite:0]{index=0}

---

## 🧾 Project Overview

The goal of this project was to build a mathematical and simulation model of a **quarter-car suspension system**, then design controllers to minimize the vibration felt by passengers.

The work includes:

- Modeling the car body (sprung mass) and wheel assembly (unsprung mass)  
- Deriving the **second-order differential equations**  
- Converting to **Laplace domain** 🚀  
- Building **transfer functions** and analyzing open-loop stability  
- Constructing the full **state-space model (A, B, C, D)**  
- Checking **controllability & observability**  
- Designing a **state-feedback controller (pole placement)**  
- Designing an **LQR controller**  
- Simulating responses in **Simulink** for a 0.1 m road disturbance  

This project represents a full pipeline from physics → math → control → simulation.  
:contentReference[oaicite:1]{index=1}

---

## 🚘 1. System Model & Physical Parameters

We modeled **two masses**:

- `m1 = 2500 kg` → vehicle body  
- `m2 = 320 kg` → wheel & axle assembly  

Springs:

- `k1 = 80,000 N/m` → main suspension  
- `k2 = 500,000 N/m` → tire stiffness  

Dampers:

- `B1 = 350 N·s/m` → suspension damping  
- `B2 = 15,020 N·s/m` → wheel/tire damping  

These are taken from the table on **page 2**.  
:contentReference[oaicite:2]{index=2}

The control objective:

> Reduce the relative displacement **(x1 − x2)** to improve passenger comfort.

---

## 🧮 2. Differential Equations (Newton’s Laws)

From page 3:

### Equation for m1 (vehicle body):

