# calibrate_accel.py
import smbus2, time

bus      = smbus2.SMBus(1)
IMU_ADDR = 0x6A
bus.write_byte_data(IMU_ADDR, 0x10, 0xC0)
bus.write_byte_data(IMU_ADDR, 0x20, 0xC0)
time.sleep(0.5)

print("Módulo plano y quieto — midiendo 50 muestras...")
sx = sy = sz = 0
N = 50
for _ in range(N):
    d  = bus.read_i2c_block_data(IMU_ADDR, 0x28 | 0x80, 6)
    ax = ((d[1]<<8)|d[0]); ax = ax-65536 if ax>=0x8000 else ax
    ay = ((d[3]<<8)|d[2]); ay = ay-65536 if ay>=0x8000 else ay
    az = ((d[5]<<8)|d[4]); az = az-65536 if az>=0x8000 else az
    sx += ax; sy += ay; sz += az
    time.sleep(0.02)


f = 0.000061 * 9.80665   # sensibilidas para ±2g
print(f"ax = {sx/N*f:.4f} m/s²  (esperado ~0)")
print(f"ay = {sy/N*f:.4f} m/s²  (esperado ~0)")
print(f"az = {sz/N*f:.4f} m/s²  (esperado ~9.8)")
