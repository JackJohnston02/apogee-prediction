import matplotlib.pyplot as plt
import numpy as np
import matplotlib

plt.close('all')

matplotlib.rcParams['text.usetex'] = True
matplotlib.rcParams['font.family'] = 'sans-serif'
matplotlib.rcParams['font.sans-serif'] = 'Arial'
matplotlib.rcParams['axes.unicode_minus'] = False  
matplotlib.rcParams['text.latex.preamble'] = r'\usepackage{sfmath}'

execution_times = {
    'Constant Acceleration': {
        'EKF': 0.543664,
        'UKF': 1.48294,
        'CKF': 1.43067
    },
    'Constant Ballistic Coefficient': {
        'EKF': 0.724686,
        'UKF': 1.98443,
        'CKF': 1.96759
    }
}

colors = {
    'EKF': "#45ae8d",
    'UKF': "#627db8",
    'CKF': "#fb6025"
}

groups = ['Constant Acceleration', 'Constant Ballistic Coefficient']
filters = ['EKF', 'UKF', 'CKF']

n_groups = len(groups)
n_filters = len(filters)

index = np.arange(n_groups) * 0.6

ekf_times = [
    execution_times['Constant Acceleration']['EKF'],
    execution_times['Constant Ballistic Coefficient']['EKF']
]

ukf_times = [
    execution_times['Constant Acceleration']['UKF'],
    execution_times['Constant Ballistic Coefficient']['UKF']
]

ckf_times = [
    execution_times['Constant Acceleration']['CKF'],
    execution_times['Constant Ballistic Coefficient']['CKF']
]

bar_width = 0.15
spacing = 0.02
total_bar_width = n_filters * bar_width + (n_filters - 1) * spacing

positions = []
for i in range(n_filters):
    pos = index - total_bar_width / 2 + i * (bar_width + spacing) + bar_width / 2
    positions.append(pos)

bar_opacity = 0.8
edge_linewidth = 1.5

fig, ax = plt.subplots(figsize=(5, 5), dpi=100)

rects1 = ax.bar(positions[0], ekf_times, bar_width, alpha=bar_opacity,
                color=colors['EKF'], label='EKF',
                edgecolor='black', linewidth=edge_linewidth)
rects2 = ax.bar(positions[1], ukf_times, bar_width, alpha=bar_opacity,
                color=colors['UKF'], label='UKF',
                edgecolor='black', linewidth=edge_linewidth)
rects3 = ax.bar(positions[2], ckf_times, bar_width, alpha=bar_opacity,
                color=colors['CKF'], label='CKF',
                edgecolor='black', linewidth=edge_linewidth)

ax.set_xlabel('Models', fontsize=12, fontweight='bold')
ax.set_ylabel('Execution Time (ms)', fontsize=12, fontweight='bold')
ax.set_title('Comparison of Filter Execution Times', fontsize=14, fontweight='bold')

ax.set_xticks(index)
ax.set_xticklabels(groups, fontsize=10)
plt.xticks(rotation=0)

max_time = max(max(ekf_times), max(ukf_times), max(ckf_times))
y_interval = 0.25
y_max = np.ceil(max_time * 1.1 / y_interval) * y_interval
ax.set_ylim(0, y_max)
ax.set_yticks(np.arange(0, y_max + y_interval, y_interval))
ax.tick_params(axis='y', which='major', labelsize=10)

ax.yaxis.set_major_formatter(plt.FormatStrFormatter('%.2f'))

ax.yaxis.grid(True, which='major', linestyle='--', linewidth=0.5, alpha=0.3)
ax.set_axisbelow(True)

for spine in ax.spines.values():
    spine.set_linewidth(1.5)

ax.tick_params(axis='both', which='major', labelsize=10, width=1.5)
ax.legend(loc='upper left', fontsize=10)

fig.tight_layout()
fig.patch.set_facecolor('white')

plt.show()