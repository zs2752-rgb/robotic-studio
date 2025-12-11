import csv
import matplotlib.pyplot as plt

logfile = "angle_log.csv"

t = []
ids = {i: [] for i in range(1, 9)}

with open(logfile, "r") as f:
    reader = csv.DictReader(f)
    for row in reader:
        t.append(float(row["t"]))
        for i in range(1, 9):
            ids[i].append(float(row[f"id{i}"]))

# 画 8 条曲线在一张图上
plt.figure(figsize=(10, 6))
for i in range(1, 9):
    plt.plot(t, ids[i], label=f"ID{i}")

plt.xlabel("Time (s)")
plt.ylabel("Angle (deg)")
plt.title("Servo angles vs time")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("angle_plot.png", dpi=300)
print("Saved as angle_plot.png")
