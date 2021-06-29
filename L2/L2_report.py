import sys

files = ["../../L2_cache/D2w_I2w__L22w_256.txt",
        "../../L2_cache/Ddm_Idm__L2dm_256.txt",
        "../../L2_cache/Ddm_Idm__L22w_256.txt",
        "../../L2_cache/D2w_I2w__L2dm_256.txt",
        "../../L2_cache/Ddm_I2w__L22w_256.txt",]
descriptions = [("2-way", "2-way", "2-way"),
                ("dm", "dm", "dm"),
                ("dm", "dm", "2-way"),
                ("2-way", "2-way", "dm"),
                ("dm", "2-way", "2-way")]

for file, descript in zip(files, descriptions):
    duration = 0
    D_r_time, D_w_time, D_stall_cycles = 0, 0, 0
    I_r_time, I_w_time, I_stall_cycles = 0, 0, 0
    with open(file, 'r') as f:
        for k in range(9):
            duration += int(f.readline().strip().split(':')[-1])
            D_r_time += int(f.readline().strip().split(':')[-1])
            D_w_time += int(f.readline().strip().split(':')[-1])
            D_stall_cycles += int(f.readline().strip().split(':')[-1])
            I_r_time += int(f.readline().strip().split(':')[-1])
            I_w_time += int(f.readline().strip().split(':')[-1])
            I_stall_cycles += int(f.readline().strip().split(':')[-1])
    print(f"\n  D: {descript[0]:6} I: {descript[1]:6} L2: {descript[2]:6}")
    print(f"  duration: {duration}", end='')
    # print(f"  D: r = {D_r_time:7},  w = {D_w_time:7},  stall = {D_stall_cycles:7}")
    # print(f"  I: r = {I_r_time:7},  w = {I_w_time:7},  stall = {I_stall_cycles:7}")
    # print(F"  avg D stalls: {(D_stall_cycles/(D_r_time+D_w_time)):2.4f},  I stalls: {(I_stall_cycles/(I_r_time+I_w_time)):2.4f}")
    print(F"  avg_stalls_total: {((D_stall_cycles+I_stall_cycles) / (D_r_time+D_w_time+I_r_time+I_w_time)):2.4f}")
print("\n\n")

#     print(f"\n  {descript}")
#     print(f"  duration: {duration}\n")
#     # print(f"  D: r = {D_r_time:7},  w = {D_w_time:7},  stall = {D_stall_cycles:7}")
#     # print(f"  I: r = {I_r_time:7},  w = {I_w_time:7},  stall = {I_stall_cycles:7}")
#     print(F"  avg D stalls: {(D_stall_cycles/(D_r_time+D_w_time)):2.4f},  I stalls: {(I_stall_cycles/(I_r_time+I_w_time)):2.4f}")
#     print(F"  avg stalls: {((D_stall_cycles+I_stall_cycles) / (D_r_time+D_w_time+I_r_time+I_w_time)):2.4f}")