import numpy as np
import matplotlib.pyplot as plt

# 1. Define physical constants and design parameters
chord_ft = 2.0
chord_m = chord_ft * 0.3048  # Convert feet to meters (0.6096 m)
nu_air = 1.46e-5            # Kinematic viscosity of air at sea level standard conditions (m²/s)
knots_to_mps = 0.514444     # Conversion factor from knots to meters per second

# 2. Generate air speed arrays
speeds_knots = np.linspace(1, 15, 200)
speeds_mps = speeds_knots * knots_to_mps

# 3. Calculate Chord Reynolds Number (Re)
reynolds_numbers = (speeds_mps * chord_m) / nu_air

# 4. Model the three distinct aerodynamic angles (in degrees)
# Max Lift-Over-Drag Angle: Safest, most efficient cruise profile
alpha_ld_max = 5.0 + 3.5 * (1.0 - np.exp(-reynolds_numbers / 150000))

# Stall Onset Angle: Where the lift curve first loses linearity due to flow separation
alpha_stall_onset = 8.5 + 4.0 * (1.0 - np.exp(-reynolds_numbers / 100000))

# Maximum Lift Angle: The absolute peak of the lift curve before deep stall
alpha_max_lift = 10.5 + 5.5 * (1.0 - np.exp(-reynolds_numbers / 130000))

# 5. Create the plot
plt.figure(figsize=(10, 6), dpi=100)

plt.plot(speeds_knots, alpha_max_lift, label='Maximum Lift Angle ($\\alpha_{Cl,max}$)', 
         color='#2ca02c', linewidth=2.5, linestyle='-')
plt.plot(speeds_knots, alpha_stall_onset, label='Stall Onset Angle ($\\alpha_{stall}$)', 
         color='#d9534f', linewidth=2.5, linestyle='--')
plt.plot(speeds_knots, alpha_ld_max, label='Max Lift-over-Drag Angle ($\\alpha_{L/D\\ max}$)', 
         color='#0275d8', linewidth=2.5, linestyle='-.')

# 6. Formatting and labels
plt.title('NACA 0018 Critical Performance Angles vs. Air Speed\n(2 ft Chord at Sea Level Standard Air)', 
          fontsize=12, fontweight='bold', pad=15)
plt.xlabel('Air Speed (knots)', fontsize=11)
plt.ylabel('Angle of Attack (degrees)', fontsize=11)
plt.xlim(1, 15)
plt.ylim(0, 18)
plt.xticks(np.arange(1, 16, 1))
plt.yticks(np.arange(0, 19, 2))
plt.grid(True, which='both', linestyle=':', alpha=0.6)
plt.legend(loc='lower right', fontsize=10, frameon=True)

# Add a top axis showing corresponding Reynolds numbers for reference
ax1 = plt.gca()
ax2 = ax1.twiny()
ax2.set_xlim(ax1.get_xlim())
tick_knots = np.arange(1, 16, 2)
ax2.set_xticks(tick_knots)
re_labels = [f'{int((k * knots_to_mps * chord_m / nu_air)/1000)}k' for k in tick_knots]
ax2.set_xticklabels(re_labels)
ax2.set_xlabel('Approximate Reynolds Number ($Re$)', fontsize=10, color='gray', labelpad=8)
ax2.tick_params(colors='gray')

plt.tight_layout()
plt.show()
