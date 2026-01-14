import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

FILE = "rpm_pwm_data.xlsx"

data = pd.read_excel(FILE)
rpm = data["RPM"].values

time = np.linspace(0, 60, len(rpm))

plt.figure(figsize=(10, 5))
plt.plot(time, rpm, label="RPM", linewidth=2)

plt.xlabel("Time (s)")
plt.ylabel("RPM")
plt.title("RPM vs Time")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
