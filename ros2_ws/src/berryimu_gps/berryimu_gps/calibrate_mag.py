#!/usr/bin/env python3
# calibrate_mag.py
import smbus2, time
bus      = smbus2.SMBus(1)
MAG_ADDR = 0x1C
bus.write_byte_data(MAG_ADDR, 0x20, 0x70)
bus.write_byte_data(MAG_ADDR, 0x22, 0x00)
time.sleep(0.5)
print("Girando lentamente el módulo 360° en plano horizontal...")
print("Ctrl+C cuando complete la vuelta\n")

MAG_SENSITIVITY = 0.14 / 1000.0  # gauss/LSB (rango ±4 gauss)

mx_min = my_min = mz_min =  99999
mx_max = my_max = mz_max = -99999
try:
    while True:
        d  = bus.read_i2c_block_data(MAG_ADDR, 0x28 | 0x80, 6)
        mx = ((d[1]<<8)|d[0]); mx = mx-65536 if mx>=0x8000 else mx
        my = ((d[3]<<8)|d[2]); my = my-65536 if my>=0x8000 else my
        mz = ((d[5]<<8)|d[4]); mz = mz-65536 if mz>=0x8000 else mz
        mx_min = min(mx_min, mx); mx_max = max(mx_max, mx)
        my_min = min(my_min, my); my_max = max(my_max, my)
        mz_min = min(mz_min, mz); mz_max = max(mz_max, mz)
        print(f"mx={mx:6d} my={my:6d} mz={mz:6d}  |  "
              f"min/max X:[{mx_min},{mx_max}] Y:[{my_min},{my_max}] Z:[{mz_min},{mz_max}]",
              end='\r')
        time.sleep(0.05)
except KeyboardInterrupt:
    ox = (mx_max + mx_min) / 2
    oy = (my_max + my_min) / 2
    oz = (mz_max + mz_min) / 2
    print(f"\n\nOffsets hard iron (gauss):")
    print(f"  MAG_OFFSET_X = {ox * MAG_SENSITIVITY:.4f}")
    print(f"  MAG_OFFSET_Y = {oy * MAG_SENSITIVITY:.4f}")
    print(f"  MAG_OFFSET_Z = {oz * MAG_SENSITIVITY:.4f}")
    
