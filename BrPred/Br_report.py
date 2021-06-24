import sys

files = ["./Br_data/bht_2bit.txt",
        "./Br_data/2bit_predictor.txt",
        "./Br_data/2bit_counter.txt"]
descriptions = ["bht_2bit", "2bit_predictor", "2bit_counter"]
# descriptions = [("2-way", "2-way", "2-way"),
#                 ("dm", "dm", "dm"),
#                 ("dm", "dm", "2-way"),
#                 ("2-way", "2-way", "dm"),
#                 ("dm", "2-way", "2-way")]

for file, descript in zip(files, descriptions):
    duration = 0
    br_times = 0
    wrong_times = 0
    with open(file, 'r') as f:
        for k in range(9):
            duration += int(f.readline().strip().split(':')[-1])
            br_times += int(f.readline().strip().split(':')[-1])
            wrong_times += int(f.readline().strip().split(':')[-1])

    print(f"\n  D: {descript[0]:6} I: {descript[1]:6} L2: {descript[2]:6}")
    # print(f"\n  D: {descript[0]:6} I: {descript[1]:6} L2: {descript[2]:6}")
    # print(f"  duration: {duration}", end='')
    print(f"  duration:{duration:6}  branch_times:{br_times:6}  wrong_times:{wrong_times}")
    print(f"  accuracy:{(1 - wrong_times/br_times):2.3f}")
    # print(f"  D: r = {D_r_time:7},  w = {D_w_time:7},  stall = {D_stall_cycles:7}")
    # print(f"  D: r = {D_r_time:7},  w = {D_w_time:7},  stall = {D_stall_cycles:7}")
    # print(f"  I: r = {I_r_time:7},  w = {I_w_time:7},  stall = {I_stall_cycles:7}")
    # print(F"  avg D stalls: {(D_stall_cycles/(D_r_time+D_w_time)):2.4f},  I stalls: {(I_stall_cycles/(I_r_time+I_w_time)):2.4f}")
    # print(F"  avg_stalls_total: {((D_stall_cycles+I_stall_cycles) / (D_r_time+D_w_time+I_r_time+I_w_time)):2.4f}")
print("\n\n")

#     print(f"\n  {descript}")
#     print(f"  duration: {duration}\n")
#     # print(f"  D: r = {D_r_time:7},  w = {D_w_time:7},  stall = {D_stall_cycles:7}")
#     # print(f"  I: r = {I_r_time:7},  w = {I_w_time:7},  stall = {I_stall_cycles:7}")
#     print(F"  avg D stalls: {(D_stall_cycles/(D_r_time+D_w_time)):2.4f},  I stalls: {(I_stall_cycles/(I_r_time+I_w_time)):2.4f}")
#     print(F"  avg stalls: {((D_stall_cycles+I_stall_cycles) / (D_r_time+D_w_time+I_r_time+I_w_time)):2.4f}")