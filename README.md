# Virtual Model Control for Free-Floating Space Robots  
**University of Cambridge – Department of Engineering (IIB MEng Project, 2025–26)**  
**Student:** Devin Karia (Clare College)  
**Supervisor:** Prof. Fulvio Forni  
**Email:** dkk35@cam.ac.uk  
**Project Type:** Type (b)

---

## 🎯 Project Overview
This project investigates the use of **Virtual Model Control (VMC)** to control a **free-floating space robot** — a spacecraft equipped with a robotic manipulator that operates without active base control.  
VMC is a physics-based control strategy that attaches *virtual mechanisms* (such as springs, dampers, and linkages) to guide the robot’s motion toward a desired pose or trajectory.

This approach has been successfully demonstrated for legged and industrial robots but not yet applied to **space robotics**, where under-actuation, base disturbances, and model uncertainty pose significant challenges.  

---

## 🧩 Objectives
1. **Derive** a virtual model controller for a free-floating space robot.  
2. **Develop** a physics-based simulation environment using PyBullet to model satellite–manipulator dynamics.  
3. **Evaluate** the controller’s robustness under system uncertainties and external perturbations.  

---

## 🧠 Background
Space robots, or *space manipulator systems*, play a crucial role in **in-orbit servicing and manufacturing (IOSM)** missions such as satellite refuelling, deorbiting, and assembly.  
So far, most missions rely on *free-flying* robots (actively controlled spacecraft bases). The *free-floating* configuration, where the spacecraft base is uncontrolled and reacts dynamically to arm motion, offers potential fuel savings but introduces complex coupled dynamics.  

This project explores whether Virtual Model Control can provide a simpler, computationally efficient alternative to advanced nonlinear or model predictive controllers typically used for free-floating manipulators.

---

## 🧪 Methodology
1. **Dynamic Modelling:**  
   Implement known equations of motion for a free-floating space robot based on multibody dynamics literature (e.g., Seddaoui & Saaj, 2021).  
2. **Simulation Environment:**  
   Develop and test models in **PyBullet**, a real-time physics simulator supporting rigid-body dynamics and joint constraints.  
3. **Virtual Model Controller Design:**  
   Define virtual forces and constraints that emulate mechanical systems (springs, dampers, etc.) to achieve stable motion control.  
4. **Performance Evaluation:**  
   Compare VMC performance against baseline control methods under uncertainty, external disturbances, and dynamic coupling.

---

## ⚙️ Software Environment
| Component | Description |
|------------|-------------|
| **Language** | Python 3.11 (Conda environment `vmc`) |
| **Physics Engine** | PyBullet |
| **Libraries** | NumPy, Matplotlib, pybullet, scipy |
| **Version Control** | Git / GitHub |
| **Documentation** | Markdown (`README.md`), Jupyter notebooks (optional) |

To record the environment:
```bash
pip freeze > requirements.txt
```
---

## References

```bibtex
@MISC{coumans2020,
  author       = {Erwin Coumans and Yunfei Bai},
  title        = {PyBullet: a Python module for physics simulation for games, robotics and machine learning},
  howpublished = {\url{http://pybullet.org}},
  year         = {2016--2023}
}

