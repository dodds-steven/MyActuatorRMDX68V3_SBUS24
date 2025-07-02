/*
 * HandleSBUS.cpp
 * 
 * Purpose:
 * This source file implements the SBUSHandler class for non-blocking SBUS data reading on a Teensy 4.1 with FrSky TD-R10.
 * It uses the bolderflight/sbus library to parse 25-byte SBUS packets (~9ms cycle, ~111Hz), delivering validated channel data
 * (0–23) in the range 172–1811 for motor control (e.g., RMD X6-8 V3). The implementation:
 * - Polls SBUS every 8.5ms, with a 1.5ms post-read wait to align with frame boundaries.
 * - Validates all channels (172–1811), trusts library’s failsafe and lostFrame flags.
 * - Flushes serial buffer to 25 bytes if >40 bytes to prevent overflow.
 * - Skips reads if <25 bytes to avoid partial frames.
 * - Centers channels (992) after 5 consecutive failed reads.
 * - Logs debug info (successes, failures, flushes, channel values) if VERBOSE_DEBUG is true.
 * 
 * Tunable Parameters (defined in HandleSBUS.h):
 * - SBUS_RETRY_INTERVAL: Read interval in ms (default 8.5, adjust 8.0–9.0 for receiver timing).
 * - SBUS_POST_READ_WAIT: Post-read wait in ms (default 1.5, adjust 1.0–2.0 for alignment).
 * - SBUS_MIN, SBUS_MAX: Channel value range (default 172, 1811, match receiver output).
 * - SBUS_CENTER: Center value (default 992, match receiver center).
 * - SBUS_DEADZONE: Normalization deadzone (default 0.05, adjust for sensitivity).
 * - VERBOSE_DEBUG: Enable/disable debug logs (default true, set to false for production).
 * 
 * Usage:
 * Instantiate SBUSHandler in main.cpp, call begin() in setup(), and poll readChannels() in loop().
 * Consumers (e.g., MotorModeController) normalize specific channels as needed.
 */

 #include "HandleSBUS.h"

 // Constructor: Default port (SBUS_PORT)
 SBUSHandler::SBUSHandler(bool verbose) 
   : serialPort(SBUS_PORT), sbus(SBUS_PORT), hasValidSBUS(false), sbusFailCount(0),
     verboseFeedback(verbose), readSuccessCount(0), readFailCount(0),
     flushCount(0), lastRead(0) {
   for (uint8_t i = 0; i < SBUS_CHANNELS; i++) lastValidChannels[i] = SBUS_CENTER;
 }
 
 // Constructor: Explicit port
 SBUSHandler::SBUSHandler(HardwareSerial& port, bool verbose) 
   : serialPort(port), sbus(port), hasValidSBUS(false), sbusFailCount(0),
     verboseFeedback(verbose), readSuccessCount(0), readFailCount(0),
     flushCount(0), lastRead(0) {
   for (uint8_t i = 0; i < SBUS_CHANNELS; i++) lastValidChannels[i] = SBUS_CENTER;
 }
 
 // Initialize serial port, clear buffer, and set up SBUS library
 void SBUSHandler::begin() {
   delay(SBUS_STARTUP_DELAY); // Wait for receiver stability
   serialPort.begin(SBUS_BAUD, SERIAL_8E2); // Start serial port with SBUS baud and parity
   while (serialPort.available()) serialPort.read(); // Clear any stale data
   sbus.begin(SBUS_RX_PIN, SBUS_TX_PIN, true, SBUS_BAUD); // Initialize SBUS with inversion
 #if VERBOSE_DEBUG
   if (verboseFeedback) {
     Serial.printf("SBUS initialized with baud rate %d, RX pin %d, TX pin %d, Parity=8E2\n",
                   SBUS_BAUD, SBUS_RX_PIN, SBUS_TX_PIN);
   }
 #endif
 }
 
 // Read SBUS channels, output channel array
 // Returns true if valid data is available, false otherwise
 bool SBUSHandler::readChannels(uint16_t* channels) {
   // Check if enough time has elapsed for next read
   if (lastRead < SBUS_RETRY_INTERVAL) {
     return hasValidSBUS;
   }
   lastRead = 0; // Reset read timer
 
   // Get available bytes in serial buffer
   int bytesAvailable = serialPort.available();
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
     // Validate all channels
     bool validRead = true;
     for (uint8_t i = 0; i < SBUS_CHANNELS; i++) {
       if (tempChannels[i] < SBUS_MIN || tempChannels[i] > SBUS_MAX) {
 #if VERBOSE_DEBUG
         if (verboseFeedback) {
           Serial.printf("SBUS: Warning, channel %d out of range: %d\n", i + 1, tempChannels[i]);
         }
 #endif
         validRead = false;
       }
     }
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
         Serial.printf("SBUS: Invalid read, failsafe=%d, lostFrame=%d, retrying %d/5\n",
                       failsafe, lostFrame, sbusFailCount);
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
   }
 
   // Flush buffer if too full to prevent overflow
   if (bytesAvailable > 40) {
     int bytesToFlush = bytesAvailable - 25;
     flushCount++; // Increment flush counter
 #if VERBOSE_DEBUG
     if (verboseFeedback) {
       Serial.printf("SBUS: Flushing %d excess bytes\n", bytesToFlush);
     }
 #endif
     for (int i = 0; i < bytesToFlush && serialPort.available(); i++) {
       serialPort.read();
     }
   }
 
   // Center channels if too many consecutive failures
   if (sbusFailCount > 5) {
 #if VERBOSE_DEBUG
     if (verboseFeedback) {
       Serial.println("SBUS: Read failed too many times, using center position (992)");
     }
 #endif
     for (uint8_t i = 0; i < SBUS_CHANNELS; i++) {
       lastValidChannels[i] = SBUS_CENTER;
     }
     hasValidSBUS = true;
     sbusFailCount = 0;
   }
 
   // Copy last valid channels to output
   memcpy(channels, lastValidChannels, sizeof(uint16_t) * SBUS_CHANNELS);
 #if VERBOSE_DEBUG
   if (verboseFeedback) {
     Serial.print("SBUS: Output channels: ");
     for (uint8_t j = 0; j < SBUS_CHANNELS; j++) {
       Serial.print(channels[j]);
       Serial.print(" ");
     }
     Serial.println();
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
 // File: HandleSBUS.cpp (203 lines)