# ActiveSpikeSystem

## 2024 SKKU Senior Capstone Design

**Active Spike System for Multi-Terrain**

---

### **Introduction**

An innovative wheel system capable of deploying spikes actively to overcome challenging terrains. This system allows smooth and high-speed driving on flat or even roads while enabling the vehicle to handle extreme terrains like ice, sand, gravel, mud, or mountainous areas by increasing ground resistance through the active deployment of spikes.

#### Features:
- Spikes can be replaced or maintained to adapt to specific ground conditions.
- Versatility for multi-terrain exploration and adaptability.


#### **Development Motivation**

- To design a wheel system capable of maintaining stability and mobility across diverse terrains.
- Address the challenges of icy, muddy, and uneven surfaces for reconnaissance or logistic operations.
- Increase reliability for exploration missions by providing active spike deployment and resistance enhancement.
---

### **Video**
![for YoutubeVideo](https://github.com/user-attachments/assets/aafca079-5c6c-4122-ac90-b65cd82237b6)

Click the image or [here](https://www.youtube.com/playlist?list=PLjpVV0HF_mcmStR3g7uam4u9qX5ewlJhn) to watch the videos.

---

### **Hardware Components**
- **4 DC Geared Motors**: For movement and steering.
- **4 Servo Motors**: For active spike deployment.
- **4 Slip Rings**: To ensure rotational connectivity for servo motors.
- **ESP32**: Manages Wi-Fi communication and GPIO control.
- **11.1V LiPo Battery**: Power supply.
- **Step-Up/Down Modules**: For voltage regulation.

---

### **Software Features**
- **WebSocket Communication**: Enables a mobile web application for controlling steering and spike deployment.

---

### **Applications**
This system has a wide range of potential applications, including:

- **Reconnaissance Robots**: Capable of navigating extreme terrains for exploration or surveillance.
- **Space Exploration Vehicles**: Adaptability to extraterrestrial terrains.
- **Military Armored Vehicles**: Enhanced mobility in combat or adverse environments.
- **Desert Logistics**: Efficient transportation across sandy terrains.


---

### **Repository Structure**
- **`/src`**: Contains the main codebase, including the logic for motor and spike system control.
- **`/data`**: Contains HTML, CSS, and JavaScript files for the web application interface.
- **`platformio.ini`**: Lists all dependencies and configurations for PlatformIO.

---

### **Libraries Used**
The following libraries are utilized in this project:

- **WiFi.h**: For ESP32 Wi-Fi functionality.
- **Arduino.h**: Core Arduino functions and definitions.
- **AsyncTCP.h**: Enables asynchronous TCP communication.
- **ESPAsyncWebServer.h**: For creating and managing an asynchronous web server.
- **SPIFFS.h**: Manages the filesystem on the ESP32 for storing web assets.
- **Wire.h**: I2C communication library for future expansions.

---

### **How It Works**
1. Smooth driving on flat terrains with no spikes deployed.
2. When challenging terrain is detected, spikes are actively deployed to increase ground grip.
3. Adjustable spikes enable maintenance and customization for specific needs.

---

### **Development Platform**
This project was developed using **PlatformIO**, an open-source ecosystem for IoT development. All dependencies and configurations are specified in the `platformio.ini` file.

---

### **Contributors**
- SKKU Senior Capstone Team 2024

---

### **Contact**
For further inquiries or collaboration opportunities, please reach out to us at [leejimmy1@g.skku.edu](mailto:leejimmy1@g.skku.edu).
