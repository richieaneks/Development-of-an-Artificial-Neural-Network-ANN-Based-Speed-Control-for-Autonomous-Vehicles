# Development of an Artificial Neural Network (ANN)-Based Speed Control System for Autonomous Vehicles

This repository contains the complete implementation, datasets, embedded C++ firmware, and performance analysis scripts developed for my Bachelor of Engineering (B.Eng) Thesis in Mechatronics Engineering at the Federal University of Technology, Minna.

The project addresses the computational constraints and non-linear tracking limitations of traditional vehicle speed controllers on low-cost hardware. It designs, trains, and physically implements an embedded Artificial Neural Network (ANN) speed control system on a custom 4WD testbed vehicle.

---

## 📄 Project Metadata

* **Author:** Richard Chidubem Aniekwe (Matric No: 2022/2/85570ET)
* **Supervisors:** Engr. Dr. Jibril Abdullahi Bala & Engr. Dr. Taliha Abiodun Folorunso
* **Institution:** Federal University of Technology, Minna, Nigeria
* **Department:** Mechatronics Engineering (DOME), School of Electrical Engineering and Technology (SEET)
* **Date:** September, 2026

---

## 📂 Repository Layout & Project Directories

The code, data, and visual performance verification plots are organized across the following system folders:

```text
├── dataset/
│   ├── ann_dataset.csv          # 2,000 synthetic state samples used for training
│   ├── telemetry.csv            # Real-time on-track data logged from the ESP32 SPIFFS
│   ├── ControlMetrics.txt       # Extracted transient response time parameters
│   └── EVALUATION_METRICS.txt   # Statistical precision values from the trained network
├── MATLAB_and_ESP32_code/
│   ├── ANN_Training_Metrics.m   # Core script for network design, Adam training, and param export
│   ├── Control_metrics.m        # Script used to parse telemetry.csv and calculate error criteria
│   └── Updated_ESP32_Firmware.ino # C++ script with the embedded network matrix for forward inference
├── objective 1/                 # ANN performance validation charts (Objective 1)
│   ├── loss_convergence.png     # MSE decay curve over 3,000 training epochs
│   ├── linear_regression.png    # R² correlation curve fit (Actual vs. Target)
│   ├── error_distribution.png   # Prediction residual histogram
│   └── control_surface.png      # 3D continuous mapping surface (e vs. de vs. PWM)
└── objective 3/                 # Physical control system validation charts (Objective 3)
    ├── step_response.png        # Time-history speed tracking curve (0.50 m/s target)
    ├── controller_effort.png    # Overlaid PWM command output vs. measured velocity
    ├── integral_errors.png      # Accumulation tracking curves for IAE, ISE, and ITAE
    └── performance_dashboard.png# Combined transient time metrics and error criteria bar chart
```

---

## 🛠️ System Design & Architecture

The autonomous testbed consists of a 4WD chassis driven by an ESP32 microcontroller and an L298N motor driver. Velocity feedback is captured through dual-channel LM393 optical slot speed sensors reading 20-slot encoder discs on the motor shafts.

### The 2-10-1 Neural Network Model

An optimized feed-forward ANN replaces traditional linear architectures. It uses a two-dimensional input vector—speed error (e) and the change in error (Δ e)—processed through a hidden layer of 10 neurons utilizing a logistic sigmoid activation function, and a single linear output neuron that outputs a Pulse Width Modulation (PWM) signal.

```text
Input Layer (2 Nodes)       Hidden Layer (10 Neurons)       Output Layer (1 Node)
    [ Error (e) ]    =====>     [ Sigmoid Logsig ]    =====>    [ Linear Purelin ] => PWM
  [ Delta Error (de) ]               Neurons                     Control Output
```

---

## 📈 Experimental Metrics & Visualizations

### Objective 1: Evaluation of the Designed Artificial Neural Network (ANN) Model

The network was trained offline using standard matrix mathematics and an Adam optimizer for 3,000 epochs. It learned to replicate an ideal Proportional-Integral (PI) control law (\(K_p = 25.0, K_i = 15.0\)).

* **Training Loss Convergence:** The model achieved rapid, stable convergence, dropping its Mean Squared Error (MSE) from over 3.0 × 10² to ~3.0 × 10⁻³ at epoch 3,000.
* **Predictive Fidelity:** Linear regression confirmed a high correlation fit (R² = 0.999988). The prediction residuals follow a zero-centered Gaussian distribution, demonstrating the model is unbiased.

### Objective 3: Performance Evaluation on Physical Hardware

Real-time, untethered testing was conducted by subjecting the 4WD autonomous prototype to a step input target speed of 0.50 m/s.

* **Transient step behavior:** The vehicle accelerated quickly, matching the target velocity within a rise time of 0.750 seconds.
* **Transient Overshoot & Settling:** Due to mechanical gear inertia and unoptimized integral parameters, the vehicle experienced a transient peak overshoot of 34.00% (reaching 0.67 m/s) and a settling time of 9.900 seconds within a ±5% error band.
* **Accumulated Integral Tracking Errors:** The system achieved a low Integral Squared Error (ISE = 0.1451) due to its quick start, while the long settling time resulted in an Integral Time-Weighted Absolute Error (ITAE) of 2.3078.

---

## 💾 System Code Summary & Deployment

### 1. Training & Analysis Scripts (`/MATLAB_and_ESP32_code`)

* **ANN_Training_Metrics.m:** Generates the training dataset, executes the Adam optimizer training loops, computes precision variables, and exports the floating-point matrices of weights and biases.
* **Control_metrics.m:** Parses the logged data array from the physical track test to compute the performance metrics and error accumulation plots.

### 2. Microcontroller Deployment Code

The C++ matrix extracted from MATLAB training is hardcoded into `Updated_ESP32_Firmware.ino`. The network's layers are parameterized as static arrays to perform fast, real-time forward propagation every 50 ms (20 Hz):

```cpp
// Extracted 2-10-1 ANN Configuration Matrix embedded in C++ Firmware
float W1[10][2] = {
    {-5.319664, -4.246876}, {-3.617298, -2.531313}, { 5.915203,  2.965242},
    {-4.547266, -2.101213}, {-4.214084, -1.956538}, {-4.560779, -3.085883},
    { 4.394485,  3.063191}, {-4.243847, -2.896430}, { 5.181577,  2.420892},
    {-7.998518, -5.771434}
};
float b1[10] = {-8.115760, -1.182491, -6.303299,  1.941213, -1.280264, 
                -3.704172, -3.641466,  0.902836,  5.661152, 11.635712};
float W2[10] = {-7.352247, -9.225656,  7.758930, -7.016056, -8.174524, 
                -7.969265,  9.983360, -8.213231,  9.354292, -6.054862};
float output_z_bias = 12.587216;
```

---

## 🚀 Quick Start Guide

### Hardware Interconnection Mapping

To build the prototype vehicle, map your connections between the ESP32, L298N driver, and speed sensors as defined in the technical report:
* **Left Motors PWM Drive:** GPIO 14 (ENA) | Forward: GPIO 27 (IN1) | Reverse: GPIO 26 (IN2)
* **Right Motors PWM Drive:** GPIO 32 (ENB) | Forward: GPIO 25 (IN3) | Reverse: GPIO 33 (IN4)
* **LM393 Encoder Interrupt Input:** GPIO 18 (D0)

### Running the System

1. Compile and upload `Updated_ESP32_Firmware.ino` using the Arduino IDE with ESP32 board support enabled.
2. Disconnect the programming cable, turn on the rocker switch, and place the car on the test track.
3. The car will autonomously track the target speed of 0.50 m/s for 10 seconds while logging data to its SPIFFS flash memory.
4. Reconnect to a PC, open the Serial Monitor at 115200 baud, and type `READ` to extract the `telemetry.csv` file.
5. Move `telemetry.csv` into the MATLAB folder and run `Control_metrics.m` to generate the system performance charts.

---

## 🏛️ Acknowledgments & Citation

This work was supervised by Engr. Dr. Jibril Abdullahi Bala and co-supervised by Engr. Dr. Taliha Abiodun Folorunso (Head of Department), Mechatronics Engineering, Federal University of Technology, Minna, Nigeria.

```text
@thesis{aniekwe2026ann,
  author       = {Aniekwe, Richard Chidubem},
  title        = {Development of an Artificial Neural Network (ANN) Based Speed Control System for Autonomous Vehicles},
  school       = {Federal University of Technology, Minna},
  department   = {Department of Mechatronics Engineering},
  year         = {2026},
  month        = {September},
  type         = {B.Eng. Thesis}
}
```
