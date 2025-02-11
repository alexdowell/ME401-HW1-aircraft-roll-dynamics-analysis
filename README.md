# ME401 HW1 Aircraft Roll Dynamics Analysis  

## Description  
This repository contains MATLAB scripts and simulations for analyzing the roll convergence dynamics of aircraft, specifically focusing on the **Navion** and **F-104 Starfighter**. The project examines roll damping stability derivatives, aileron control effectiveness, maximum roll rate, and open-loop step response across various flight conditions.  

## Files Included  

### **Main Analysis Script**  
- **File:** ME_401_0007_HW1.m  
- **Topics Covered:**  
  - Computation of roll damping stability derivatives (L_p).  
  - Analysis of aileron control effectiveness (L_da).  
  - Maximum roll rate estimation under different airspeeds.  
  - Step response simulations of roll convergence.  

### **Step Response Analysis**  
- **File:** ME_401_0007_HW1_Step.m  
- **Topics Covered:**  
  - Open-loop step response evaluation.  
  - Comparison of roll dynamics for Navion and F-104 Starfighter.  
  - Impact of changes in control input magnitude on roll behavior.  

### **Practice Script for Additional Testing**  
- **File:** ME_401_0007_HW1_practice.m  
- **Contents:**  
  - Additional calculations for validating roll rate behavior.  
  - Tests for different stability conditions and control effectiveness.  

### **Project Report**  
- **File:** ME_401-007_HW1.pdf  
- **Contents:**  
  - Explanation of roll damping effects at different airspeeds.  
  - Graphical analysis of max roll rate and aileron effectiveness.  
  - Discussion of fuel-in-wingtip effects on roll behavior.  

## Installation  
Ensure MATLAB is installed before running the scripts.  

### Required MATLAB Toolboxes  
- Aerospace Toolbox  
- Control System Toolbox  

## Usage  
1. Open MATLAB and navigate to the repository folder.  
2. Run the main script using:  
   ME_401_0007_HW1.m  
3. Execute the step response analysis script using:  
   ME_401_0007_HW1_Step.m  
4. Review generated figures, including:  
   - Stability derivatives as functions of airspeed.  
   - Maximum roll rate comparisons.  
   - Step response simulations.  
   - Impact of aileron failures on roll behavior.  

## Example Output  

- **Roll Damping Stability Derivatives (L_p) vs. Airspeed:**  
  - L_p values decrease as airspeed increases, stabilizing roll response.  
  - Higher speeds result in greater roll stability.  

- **Max Roll Rate vs. Airspeed:**  
  - F-104 achieves a significantly higher roll rate compared to Navion.  
  - Aileron effectiveness diminishes at higher speeds.  

- **Open-Loop Step Response Comparison:**  
  - Navion exhibits slower roll convergence.  
  - F-104 has a rapid response with higher roll acceleration.  

## Contributions  
This repository is designed for educational and research purposes. Feel free to fork and modify the scripts.  

## License  
This project is open for academic and research use.  

---
**Author:** Alexander Dowell  
