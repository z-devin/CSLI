import time
import spidev

# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Device 0
spi.max_speed_hz = 100000  # Very low speed for testing
spi.mode = 0

def read_adc(channel):
    # MCP3008 command format
    cmd = [1, (8 + channel) << 4, 0]
    resp = spi.xfer2(cmd)
    
    # Extract 10-bit value
    adc_value = ((resp[1] & 3) << 8) + resp[2]
    
    # Print raw value and command/response for debugging
    print(f"Channel {channel}: Command: {cmd}, Response: {resp}, Value: {adc_value}")
    
    return adc_value

# Test read from each channel
try:
    print("Testing MCP3008 SPI communication...")
    for i in range(8):  # MCP3008 has 8 channels (0-7)
        value = read_adc(i)
        voltage = value * 3.3 / 1023
        print(f"Channel {i}: Raw value = {value}, Voltage = {voltage:.2f}V")
        time.sleep(0.5)
        
    # Read channel 0 multiple times to check consistency
    print("\nReading channel 0 ten times:")
    for i in range(10):
        value = read_adc(0)
        voltage = value * 3.3 / 1023
        print(f"Reading {i+1}: Raw value = {value}, Voltage = {voltage:.2f}V")
        time.sleep(0.2)
        
finally:
    spi.close()
    print("SPI connection closed")