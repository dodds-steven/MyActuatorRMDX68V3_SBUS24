/*
 * HandleSBUS.cpp
 * 
 * Purpose:
 * This source file implements the SBUSHandler class for non-blocking SBUS data reading on a Teensy 4.1 with FrSky TD-R10.
 * It uses the bolderflight/sbus library to parse 25-byte SBUS packets (~9ms cycle, ~111Hz), delivering validated channel data
 * (X CH11, Y CH10) in the range 172–1811, normalized to -1.0 to 1.0, for motor control (e.g., RMD X6-8 V3). The implementation:
 * - Polls SBUS every 8.5ms, with a 1.5ms post-read wait to align with frame boundaries.
 * - Validates X, Y channels (172–1811), trusts library’s failsafe and lostFrame flags.
 * - Flushes Serial1 buffer to 25 bytes if >40 bytes to prevent overflow.
 * - Skips reads if <25 bytes to avoid partial frames.
 * - Centers channels (992) after 5 consecutive failed reads.
 * - Logs debug info (successes, failures, flushes, channel values) if VERBOSE_DEBUG is 1.
 * 
 * Tunable Parameters (defined in HandleSBUS.h):
 * - SBUS_RETRY_INTERVAL: Read interval in ms (default 8.5, adjust 8.0–9.0 for receiver timing).
 * - SBUS_POST_READ_WAIT: Post-read wait in ms (default 1.5, adjust 1.0–2.0 for alignment).
 * - SBUS_MIN, SBUS_MAX: Channel value range (default 172, 1811, match receiver output).
 * - SBUS_CENTER: Center value (default 992, match receiver center).
 * - SBUS_DEADZONE: Normalization deadzone (default 0.05, adjust for sensitivity).
 * - VERBOSE_DEBUG: Enable/disable debug logs (default 1, set to 0 for production).
 * 
 * Usage:
 * Instantiate SBUSHandler in main.cpp, call begin() in setup(), and poll readChannels() in loop().
 * Use xNorm, yNorm for motor control. Tune parameters for different SBUS receivers or performance needs.
 * Monitor debug logs to verify success rate (>99.76%) and adjust if needed.
 */

 #include "HandleSBUS.h"

 // Constructor: Initialize SBUS instance and set default channel values to center
 SBUSHandler::SBUSHandler(bool verbose) : sbus(Serial1), hasValidSBUS(false), sbusFailCount(0),
                                         verboseFeedback(verbose), readSuccessCount(0), readFailCount(0),
                                         flushCount(0), lastRead(0) {
   for (uint8_t i = 0; i < SBUS_CHANNELS; i++) lastValidChannels[i] = SBUS_CENTER;
 }
 
 // Initialize Serial1, clear buffer, and set up SBUS library
 void SBUSHandler::begin() {
   delay(SBUS_STARTUP_DELAY); // Wait for receiver stability
   Serial1.begin(SBUS_BAUD, SERIAL_8E2); // Start Serial1 with SBUS baud and parity
   while (Serial1.available()) Serial1.read(); // Clear any stale data
   sbus.begin(SBUS_RX_PIN, SBUS_TX_PIN, true, SBUS_BAUD); // Initialize SBUS with inversion
   sbus.setEndPoints(X_CHANNEL, SBUS_MIN, SBUS_MAX); // Set X channel range
   sbus.setEndPoints(Y_CHANNEL, SBUS_MIN, SBUS_MAX); // Set Y channel range
 #if VERBOSE_DEBUG
   if (verboseFeedback) {
     Serial.println("SBUS initialized with baud rate 100000, RX pin 0, TX pin 1, Parity=8E2");
   }
 #endif
 }
 
 // Read SBUS channels, output normalized X, Y values, and channel array
 // Returns true if valid data is available, false otherwise
 bool SBUSHandler::readChannels(float& xNorm, float& yNorm, uint16_t* channels) {
   // Check if enough time has elapsed for next read
   if (lastRead < SBUS_RETRY_INTERVAL) {
     return hasValidSBUS;
   }
   lastRead = 0; // Reset read timer
 
   // Get available bytes in Serial1 buffer
   int bytesAvailable = Serial1.available();
 #if VERBOSE_DEBUG
   if (verboseFeedback) {
     Serial.printf("SBUS: Attempting read, bytes available: %d\n", bytesAvailable);
   }
 #endif
 
   // Skip read if insufficient bytes for a full SBUS frame
   if (bytesAvailable < 25) {
 #if VERBOSE_DEBUG
     if (verboseFeedback) {
       Serial.println("SBUS: Skipping read, insufficient bytes");
     }
 #endif
     return hasValidSBUS;
   }
 
   // Temporary array to store new channel data
   uint16_t tempChannels[SBUS_CHANNELS];
   bool failsafe, lostFrame;
   // Attempt to read SBUS packet
   if (sbus.read(&tempChannels[0], &failsafe, &lostFrame)) {
     // Validate X and Y channels
     bool validRead = (tempChannels[X_CHANNEL] >= SBUS_MIN && tempChannels[X_CHANNEL] <= SBUS_MAX) &&
                      (tempChannels[Y_CHANNEL] >= SBUS_MIN && tempChannels[Y_CHANNEL] <= SBUS_MAX);
 #if VERBOSE_DEBUG
     // Log warnings for out-of-range non-critical channels
     for (uint8_t i = 0; i < SBUS_CHANNELS; i++) {
       if (i != X_CHANNEL && i != Y_CHANNEL && (tempChannels[i] < SBUS_MIN || tempChannels[i] > SBUS_MAX)) {
         if (verboseFeedback) {
           Serial.printf("SBUS: Warning, channel %d out of range: %d\n", i + 1, tempChannels[i]);
         }
       }
     }
 #endif
     // Check if read is valid and no failsafe or lost frame
     if (validRead && !failsafe && !lostFrame) {
       readSuccessCount++; // Increment success counter
       sbusFailCount = 0; // Reset failure counter
 #if VERBOSE_DEBUG
       if (verboseFeedback) {
         Serial.printf("SBUS: Read success, failsafe=%d, lostFrame=%d\n", failsafe, lostFrame);
         Serial.print("Raw channels: ");
         for (uint8_t j = 0; j < SBUS_CHANNELS; j++) {
           Serial.print(tempChannels[j]);
           Serial.print(" ");
         }
         Serial.println();
       }
 #endif
       hasValidSBUS = true; // Mark data as valid
       memcpy(lastValidChannels, tempChannels, sizeof(uint16_t) * SBUS_CHANNELS); // Store valid data
       lastRead -= (unsigned long)(SBUS_POST_READ_WAIT * 1000); // Apply post-read wait in microseconds
     } else {
       readFailCount++; // Increment failure counter
       sbusFailCount++; // Increment consecutive failure counter
 #if VERBOSE_DEBUG
       if (verboseFeedback) {
         Serial.printf("SBUS: Invalid read, X=%d, Y=%d, failsafe=%d, lostFrame=%d, retrying %d/5\n",
                       tempChannels[X_CHANNEL], tempChannels[Y_CHANNEL], failsafe, lostFrame, sbusFailCount);
       }
 #endif
     }
   } else {
     readFailCount++; // Increment failure counter
     sbusFailCount++; // Increment consecutive failure counter
 #if VERBOSE_DEBUG
     if (verboseFeedback) {
       Serial.printf("SBUS: Read failed, retrying %d/5, bytes available: %d\n",
                     sbusFailCount, bytesAvailable);
     }
 #endif
     // Flush buffer if too full to prevent overflow
     if (bytesAvailable > 40) {
       int bytesToFlush = bytesAvailable - 25;
       flushCount++; // Increment flush counter
 #if VERBOSE_DEBUG
       if (verboseFeedback) {
         Serial.printf("SBUS: Flushing %d excess bytes\n", bytesToFlush);
       }
 #endif
       for (int i = 0; i < bytesToFlush && Serial1.available(); i++) {
         Serial1.read();
       }
     }
   }
 
   // Center channels if too many consecutive failures
   if (sbusFailCount > 5) {
 #if VERBOSE_DEBUG
     if (verboseFeedback) {
       Serial.println("SBUS: Read failed too many times, using center position (992, 992)");
     }
 #endif
     lastValidChannels[X_CHANNEL] = SBUS_CENTER;
     lastValidChannels[Y_CHANNEL] = SBUS_CENTER;
     hasValidSBUS = true;
     sbusFailCount = 0;
   }
 
   // Copy last valid channels to output
   memcpy(channels, lastValidChannels, sizeof(uint16_t) * SBUS_CHANNELS);
   // Normalize X and Y values
   xNorm = normalizeSbus(channels[X_CHANNEL]);
   yNorm = normalizeSbus(channels[Y_CHANNEL]);
 #if VERBOSE_DEBUG
   if (verboseFeedback) {
     Serial.printf("SBUS: Normalized X=%f, Y=%f, X_raw=%d, Y_raw=%d\n",
                   xNorm, yNorm, channels[X_CHANNEL], channels[Y_CHANNEL]);
   }
 #endif
   return hasValidSBUS;
 }
 
 // Normalize SBUS value to -1.0 to 1.0 with deadzone
 float SBUSHandler::normalizeSbus(uint16_t sbusValue) {
   sbusValue = constrain(sbusValue, SBUS_MIN, SBUS_MAX);
   float norm = (float)(sbusValue - SBUS_CENTER) / (SBUS_MAX - SBUS_CENTER);
   if (abs(norm) < SBUS_DEADZONE) norm = 0.0;
   return constrain(norm, -1.0, 1.0);
 }
 
 // Print test summary with success, failure, and flush counts
 void SBUSHandler::printSummary() {
 #if VERBOSE_DEBUG
   if (verboseFeedback) {
     Serial.printf("SBUS: Test summary - Successes: %d, Failures: %d, Flushes: %d\n",
                   readSuccessCount, readFailCount, flushCount);
   }
 #endif
 }