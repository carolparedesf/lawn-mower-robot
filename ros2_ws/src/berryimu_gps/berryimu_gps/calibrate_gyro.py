#!/usr/bin/env python3
"""
Calibracion de bias del giroscopio
"""
import smbus2, time
bus      = smbus2.SMBus(1)
IMU_ADDR = 0x6A
bus.write_byte_data(IMU_ADDR, 0x10, 0xC0)
bus.write_byte_data(IMU_ADDR, 0x20, 0xC0)
time.sleep(0.5)
def read_block(addr, reg):
    d = bus.read_i2c_block_data(addr, reg | 0x80, 6)
    vals = []
    for i in range(0, 6, 2):
        w = (d[i+1] << 8) | d[i]
        vals.append(w - 65536 if w >= 0x8000 else w)
    return vals
print("Mantenga el módulo completamente quieto...")
print("Midiendo bias del giroscopio (100 muestras)...")
sx = sy = sz = 0
N = 100
for _ in range(N):
    gx, gy, gz = read_block(IMU_ADDR, 0x18)
    sx += gx; sy += gy; sz += gz
    time.sleep(0.01)

GYRO_SENSITIVITY = 8.75 / 1000.0  # dps/LSB para ±245dps (solo para mostrar referencia)

bias_x_lsb = sx / N
bias_y_lsb = sy / N
bias_z_lsb = sz / N

print(f"\nBias gyro (LSB crudo, unidad que espera imu_node.py):")
print(f"  gyro_bias_x: {bias_x_lsb:.2f}")
print(f"  gyro_bias_y: {bias_y_lsb:.2f}")
print(f"  gyro_bias_z: {bias_z_lsb:.2f}")

