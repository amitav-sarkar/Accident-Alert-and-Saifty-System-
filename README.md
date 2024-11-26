# Accident-Alert-and-Saifty-System-
ACCIDENT DETECTION AND ALERT SYSTEM ,Location Tracking System Using TTGO T-Call ESP32, SIM800L GSM Module, GPS, and MPU-6050 Accelerometer

Abstract

Road accidents remain a major global concern, necessitating real-time systems for accident detection and alerting emergency services. This paper presents a comprehensive accident alert and location tracking system leveraging TTGO T-Call ESP32, SIM800L GSM module, GPS, and the MPU-6050 accelerometer. Key features include accident detection through accelerometer-based orientation changes, speed monitoring via GPS, manual alert functionality, and communication via GSM. The system is designed with user-centric features like status indicators, over-speed detection alerts, and power optimizations. Real-world tests validate its efficacy in emergency scenarios.

 

I. Introduction

The increase in vehicular accidents highlights the need for robust, automated systems capable of providing timely emergency assistance. Traditional methods rely heavily on human intervention, often resulting in delayed response times. This project aims to address these issues by integrating hardware and software to create an automated accident alert system that immediately notifies emergency services with location details.

 

II. System Overview

A. Hardware Components

1.	TTGO T-Call ESP32:

Integrates GSM (SIM800L) and GPS modules, enabling communication and location tracking.

2.	MPU-6050 Accelerometer:

Detects sudden motion changes and orientation shifts indicative of accidents.

3.	SIM800L GSM Module:

Sends SMS and places calls to predefined emergency contacts.

4.	GPS Module:

Tracks speed, latitude, and longitude using TinyGPS++ library.

5.	Relays, LEDs, and Buzzer:

Provide visual and auditory alerts for over-speeding and accident detection.

B. Software Framework

The system uses the Arduino IDE with libraries for GSM communication, GPS data parsing, and MPU-6050 motion sensing. It employs modularized code for enhanced readability and error handling.

 

III. Features and Functionality

A. Emergency Detection and Alert

The MPU-6050 detects potential accidents based on sudden changes in acceleration and orientation. Upon detection:

•	An SMS is sent with location details.

•	A call is initiated to a predefined emergency contact.

B. Over-speed Detection

Real-time GPS data monitors vehicle speed. If the speed exceeds 80 km/h:

•	LEDs and a buzzer are activated.

•	Relays trigger additional safety mechanisms.

C. Manual Alerts

A physical button allows users to manually send safety notifications.

D. Status Indicators

LEDs indicate:

•	GSM module status.

•	GPS signal acquisition.

•	Accelerometer connectivity.

 

IV. Optimizations

1.	Debouncing Button Inputs

Software debouncing ensures stable detection of button presses, preventing false triggers.

2.	Power Management

The system enters low-power modes during inactivity to optimize battery consumption.

3.	Error Handling

Retries and error messages are implemented for GSM and GPS initialization.

4.	Safety Enhancements

•	Alerts are sent to multiple emergency numbers.

•	Emergency contacts are stored in EEPROM or SPIFFS, allowing updates without reprogramming.

 

V. Testing and Calibration

A. Sensitivity Testing

Real-world tests ensure the MPU-6050 reliably detects accident scenarios. Thresholds for acceleration and orientation changes are calibrated based on practical conditions.

B. Speed Monitoring

GPS speed thresholds are validated on different terrains and vehicle types.

 

VI. Results and Discussion

The proposed system demonstrated reliable performance in detecting accidents and sending alerts. Over-speed detection was consistent, and the system's modularity enhanced maintainability. Power optimizations extended battery life in field tests.

 

VII. Conclusion and Future Work

This accident alert system effectively combines low-cost hardware and real-time software to address vehicular safety challenges. Future work includes:

•	Integrating AI for predictive accident analysis.

•	Enhancing GPS accuracy in urban environments.

•	Developing a mobile application interface for alert monitoring.

 

References

1.	A. Kumar, et al., "Accident Detection and Notification System Using IoT," International Journal of Advanced Research, 2023.

2.	H. Singh, et al., "GSM and GPS Based Vehicular Alert System," IEEE Transactions on Consumer Electronics, 2022.

3.	TinyGPS++ Documentation, "Efficient GPS Parsing Library," 2024.



