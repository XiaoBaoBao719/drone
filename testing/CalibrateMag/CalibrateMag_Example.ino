// #include <Wire.h>
// #include <src/Magnetometer.h>

// // Magnetic declination for your location (degrees)
// // Get from: https://www.magnetic-declination.com
// constexpr float MAG_DECLINATION = 0.0;

// Magnetometer magnetometer;

// void setup() {
//     Serial.begin(115200);
//     delay(1000);
    
//     Serial.println("=== MLX90393 Magnetometer Calibration ===");
//     Wire.begin();
    
//     // Initialize magnetometer
//     if (!magnetometer.begin()) {
//         Serial.println("Failed to initialize magnetometer!");
//         while (1) {
//             digitalWrite(LED_BUILTIN, HIGH);
//             delay(100);
//             digitalWrite(LED_BUILTIN, LOW);
//             delay(100);
//         }
//     }
    
//     // Configure sensor parameters
//     magnetometer.configureSensor(
//         1,    // Gain selection (1)
//         0, 0, 0,  // Resolution for X, Y, Z axes
//         0,    // Oversampling (0)
//         0     // Digital filtering (0)
//     );
    
//     Serial.println("Sensor configured. Starting calibration...");
//     Serial.println("Please rotate the sensor in all directions for accurate calibration.");
//     delay(2000);
    
//     /* ++++ Collect magnetometer data ++++ */
//     Serial.println("Collecting magnetometer samples...");
//     const int NUM_SAMPLES = 500;
    
//     if (!magnetometer.readRawData(NUM_SAMPLES)) {
//         Serial.println("Failed to read magnetometer data!");
//         return;
//     }
    
//     Serial.println("Data collection complete.");
    
//     /* ++++ Compute hard-iron offsets ++++ */
//     Serial.println("\nComputing hard-iron offsets...");
//     BLA::Matrix<3, 1> hard_iron_offsets = magnetometer.computeHardIronOffsets(NUM_SAMPLES);
    
//     Serial.println("Hard-iron offsets (mT):");
//     Serial.print("X: "); Serial.print(hard_iron_offsets(0), 4);
//     Serial.print("  Y: "); Serial.print(hard_iron_offsets(1), 4);
//     Serial.print("  Z: "); Serial.println(hard_iron_offsets(2), 4);
    
//     /* ++++ Compute soft-iron correction matrix ++++ */
//     Serial.println("\nComputing soft-iron correction matrix...");
//     BLA::Matrix<3, 3> soft_iron_matrix = magnetometer.computeSoftIronOffsets(NUM_SAMPLES);
    
//     Serial.println("Soft-iron correction matrix:");
//     for (int i = 0; i < 3; i++) {
//         for (int j = 0; j < 3; j++) {
//             Serial.print(soft_iron_matrix(i, j), 6);
//             Serial.print("  ");
//         }
//         Serial.println();
//     }
    
//     // Get calibration data
//     Magnetometer::CalibrationData calib = magnetometer.getCalibrationData();
//     Serial.println("\nEigenvalues:");
//     Serial.print("λ1: "); Serial.print(calib.eigen_values(0), 6);
//     Serial.print("  λ2: "); Serial.print(calib.eigen_values(1), 6);
//     Serial.print("  λ3: "); Serial.println(calib.eigen_values(2), 6);
    
//     Serial.println("\n=== Calibration Complete ===");
//     Serial.println("Ready for heading computation.");
//     delay(2000);
// }

// void loop() {
//     // Read calibrated magnetometer data
//     BLA::Matrix<3, 1> mag_calibrated = magnetometer.readCalibratedData();
    
//     // Compute magnetic heading
//     float heading = magnetometer.computeHeading(mag_calibrated, MAG_DECLINATION);
    
//     // Display results
//     Serial.print("Mag (mT): [");
//     Serial.print(mag_calibrated(0), 2); Serial.print(", ");
//     Serial.print(mag_calibrated(1), 2); Serial.print(", ");
//     Serial.print(mag_calibrated(2), 2);
//     Serial.print("]  Heading: ");
//     Serial.print(heading, 1);
//     Serial.println("°");
    
//     delay(100);
// }
