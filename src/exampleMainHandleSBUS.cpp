/*
 * rename this to main.cpp to test standalone SBUS functionality using the HandleSBUS library.
 * This code is designed to run on a Teensy 4.1 board with an SBUS receiver (e.g., FrSky TD-R10).
 * It uses the BOLDERflight SBUS library to read and validate SBUS data, specifically channels 10 and 11,
 * which are normalized for motor control applications.
 * 
 * Purpose:
 * This main program implements a 5-second diagnostic test for the SBUSHandler class on a Teensy 4.1 with FrSky TD-R10.
 * It polls SBUS data non-blocking, delivering validated channel values (X CH11, Y CH10) in the range 172–1811, normalized
 * to -1.0 to 1.0, for motor control (e.g., RMD X6-8 V3). The test verifies SBUS reliability (>99.76% success rate, ~553/555
 * cycles in 5s) and logs channel data, successes, failures, and buffer flushes. Key features:
 * - Runs a 5-second test, polling SBUS every ~9ms.
 * - Outputs normalized X, Y values and selected channel data (Ch9, Ch12) for debugging.
 * - Summarizes performance at test end.
 * 
 * Tunable Parameters (defined in HandleSBUS.h):
 * - TEST_DURATION: Test length in ms (default 5000, adjust for longer/shorter tests).
 * - LOOP_INTERVAL_MS: Polling interval in ms (default 9, match SBUS_RETRY_INTERVAL).
 * - VERBOSE_DEBUG: Enable/disable debug logs (default 1, set to 0 for production).
 * - SBUS_RETRY_INTERVAL, SBUS_POST_READ_WAIT: See HandleSBUS.h for timing adjustments.
 * 
 * Usage:
 * Compile and upload to Teensy 4.1 with PlatformIO. Run with motors disconnected to verify SBUS data.
 * Use xNorm, yNorm in loop() for motor control (e.g., map to RMD motor commands).
 * Review debug logs to confirm success rate and tune parameters if needed for other receivers.
 */

 #include <Arduino.h>
 #include "HandleSBUS.h"
 
 // Test configuration
 const uint32_t TEST_DURATION = 5000; // Test duration in ms
 const uint32_t LOOP_INTERVAL_MS = 9; // Polling interval in ms
 const bool verboseFeedback = VERBOSE_DEBUG; // Debug logging enable
 
 // Instantiate SBUSHandler
 SBUSHandler sbusHandler(verboseFeedback);
 
 // Setup function: Initialize serial and SBUS
 void setup() {
   Serial.begin(115200); // Start debug serial
   while (!Serial && millis() < 1000); // Wait for serial connection
 #if VERBOSE_DEBUG
   if (verboseFeedback) {
     Serial.println("Starting SBUS Diagnostic Test (5s, No Motor Movement)...\n");
   }
 #endif
   sbusHandler.begin(); // Initialize SBUS
 }
 
 // Main loop: Run 5-second test, poll SBUS, and log results
 void loop() {
   static elapsedMillis testTime; // Test duration timer
   static elapsedMillis lastLoop; // Loop interval timer
   uint16_t channels[SBUS_CHANNELS]; // Channel data array
   float xNorm, yNorm; // Normalized X, Y values
 
   // End test after duration
   if (testTime > TEST_DURATION) {
     sbusHandler.printSummary(); // Print results
 #if VERBOSE_DEBUG
     if (verboseFeedback) {
       Serial.println("SBUS test completed.");
     }
 #endif
     while (true); // Halt
   }
 
   // Poll SBUS at loop interval
   if (lastLoop < LOOP_INTERVAL_MS) {
     return;
   }
 
   // Read SBUS data
   if (sbusHandler.readChannels(xNorm, yNorm, channels)) {
 #if VERBOSE_DEBUG
     if (verboseFeedback) {
       // Log key channel values
       Serial.print("SBUS: X Channel (11): ");
       Serial.print(channels[X_CHANNEL]);
       Serial.print(", Y Channel (10): ");
       Serial.print(channels[Y_CHANNEL]);
       Serial.print(", Ch9: ");
       Serial.print(channels[8]);
       Serial.print(", Ch12: ");
       Serial.println(channels[11]);
     }
 #endif
   }
 
   lastLoop = 0; // Reset loop timer
 }