# 🧠 Stress Detection using TinyML on ESP32

This project detects human stress levels using physiological sensor data (e.g., ECG, EDA, Heart Rate) and deploys a trained machine learning model on an ESP32 microcontroller using TinyML.

---

## 📘 Overview
The project combines machine learning and embedded systems to perform **real-time stress detection** on a low-power IoT device.  
A **Random Forest model** was trained for classification and later converted into a TinyML-compatible format using `emlearn`.

---

## ⚙️ System Architecture
1. **Data Collection** – Physiological signals captured (ECG/EDA/HR)
2. **Model Training** – Random Forest classifier trained (97.4% accuracy)
3. **TinyML Conversion** – Model converted to C header using `emlearn`
4. **Deployment** – Model embedded and deployed on ESP32
5. **Output** – Real-time stress prediction displayed via Serial Monitor or BLE

---

## 🧩 Model Implementation & Conversion Notebook

The notebook covers:
- Data preprocessing  
- Feature extraction  
- Model training (Random Forest)  
- TinyML conversion using `emlearn`  
- Accuracy comparison before and after conversion  

---

## 💻 Arduino / ESP32 Deployment
All deployment code is inside the [`arduino_code`](./arduino_code) folder.

**Key Files:**
- `main.ino` → Main ESP32 program  
- `stress_model_emlearn.h` → C-converted TinyML model  
- `utils.cpp` → Helper functions for preprocessing  

**Output Example:**
```text
Features: [0.35, 0.42, 0.55, ...]
Predicted Stress Level: HIGH
