#include <Wire.h>

const uint8_t I2C_ADDRESS = 0x12;  // Fixed I2C address

void setup() {
    Serial.begin(9600);  // Initialize serial communication
    Wire.begin();        // Initialize I2C communication as master
    Serial.println("Enter comma-separated data to write (e.g., 8,1,1,1,1,1,1,1 to write values starting at register 8), or 'read,1-8' to read registers 1 to 8:");
}

void loop() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');  // Read input from serial until newline
        input.trim();  // Remove any trailing whitespace
        
        // Check if the user wants to read a range of registers
        if (input.startsWith("read,")) {
            if (input.indexOf('-') != -1) {  // Check if there's a range
                int startRegister = input.substring(5, input.indexOf('-')).toInt();
                int endRegister = input.substring(input.indexOf('-') + 1).toInt();
                readRangeFromI2C(startRegister, endRegister);  // Read data from specified range
            } else {
                int registerAddress = input.substring(5).toInt();  // Extract the register address after "read,"
                readFromI2C(registerAddress);  // Read data from the specified register
            }
        } else {
            writeToI2C(input);  // Write the input data to I2C
        }
    }
}

void writeToI2C(String dataInput) {
    char dataBuffer[50];  // Buffer to store the input (increase size if needed)
    dataInput.toCharArray(dataBuffer, 50);  // Convert the String to a char array
    
    // Split the input by commas
    char *token = strtok(dataBuffer, ",");
    int startRegister = atoi(token);  // Get the starting register from the first token
    int index = 0;

    Wire.beginTransmission(I2C_ADDRESS);  // Start transmission to the fixed I2C address
    while ((token = strtok(NULL, ",")) != NULL) {  // Get the next token
        int value = atoi(token);  // Convert each token to an integer
        
        // Write to the respective register
        Wire.write(startRegister + index);  // Write the current register address
        Wire.write((value >> 24) & 0xFF);  // Write the most significant byte
        Wire.write((value >> 16) & 0xFF);  // Write the second byte
        Wire.write((value >> 8) & 0xFF);   // Write the third byte
        Wire.write(value & 0xFF);          // Write the least significant byte
        index++;  // Increment index for next register
        
        if (index >= 8) {  // Stop after writing 8 registers (up to register 16)
            break;
        }
    }
    
    Wire.endTransmission();  // End transmission
    Serial.println("Data written to I2C.");
    Serial.println("Enter more data or press Enter to continue:");
}

void readFromI2C(int registerAddress) {
    // Step 1: Write the register address to the slave
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(registerAddress);  // Send the register address to read from
    Wire.endTransmission(false);  // Send STOP condition (false means repeated start)

    // Step 2: Now request 4 bytes of data (since each register is a uint32_t)
    Wire.requestFrom(I2C_ADDRESS, sizeof(uint32_t));  // Request 4 bytes from the slave device

    if (Wire.available() == sizeof(uint32_t)) {
        uint32_t data = 0;
        data |= Wire.read();        // Read the first byte (least significant)
        data |= Wire.read() << 8;   // Read the second byte
        data |= Wire.read() << 16;  // Read the third byte
        data |= Wire.read() << 24;  // Read the fourth byte (most significant)
        
        Serial.print("Data from register ");
        Serial.print(registerAddress);
        Serial.print(": ");
        Serial.println(data);
    } else {
        Serial.println("Failed to receive 4 bytes from I2C.");
    }

    Serial.println("Enter more data or press Enter to continue:");
}

void readRangeFromI2C(int startRegister, int endRegister) {
    for (int registerAddress = startRegister; registerAddress <= endRegister; registerAddress++) {
        readFromI2C(registerAddress);  // Call read function for each register in the range
    }
    Serial.println("Completed reading register range.");
}
