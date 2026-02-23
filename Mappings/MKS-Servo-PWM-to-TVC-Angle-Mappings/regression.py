
import numpy as np
import pandas as pd

# Read the CSV file
data = pd.read_csv('./x_servo_mapping_v1.csv', header=None)

# Extract PWM (column A/0) and Angle (column C/2)
pwm = data.iloc[:, 1].values
pwm = pwm[1:]
angle = data.iloc[:, 3].values
angle = angle[1:]
print(pwm)
print(angle)
print(f"Number of data points: {len(pwm)}")
print(f"Angle range: {angle.min():.2f} to {angle.max():.2f} degrees")
print(f"PWM range: {pwm.min()} to {pwm.max()}")

# Fit a 2nd order polynomial: PWM = P0 + P1*angle + P2*angle^2
# Using numpy polyfit (degree 2)
coefficients = np.polyfit(angle, pwm, 2)
print(coefficients)
P0 = coefficients[0]  # coefficient of angle^2
P1 = coefficients[1]  # coefficient of angle
P2 = coefficients[2]  # constant term

print("\n" + "="*60)
print("QUADRATIC POLYNOMIAL COEFFICIENTS")
print("="*60)
print(f"PWM = P3 + P2*angle + P1*angle^2")
print(f"\nP3 = {P0:.10f}")
print(f"P2 = {P1:.10f}")
print(f"P1 = {P2:.10f}")
print("="*60)

# Calculate R-squared to show fit quality
y_pred = P0 + P1*angle + P2*angle**2
ss_res = np.sum((pwm - y_pred)**2)
ss_tot = np.sum((pwm - np.mean(pwm))**2)
r_squared = 1 - (ss_res / ss_tot)

print(f"\nR² (coefficient of determination): {r_squared:.6f}")
print(f"This means the model explains {r_squared*100:.2f}% of the variance")

# Show a few test predictions
print("\n" + "="*60)
print("TEST PREDICTIONS:")
print("="*60)
test_angles = [-8, 0, 5, 10]
for test_angle in test_angles:
    predicted_pwm = P0 + P1*test_angle + P2*test_angle**2
    print(f"Angle = {test_angle:6.1f}° → PWM = {predicted_pwm:.1f}")
