# Given the projectile y = x * tan(theta) - (g * x^2) / (2 * v^2 * cos^2(theta))
# And the distance is known, d = v^2 * sin(2 * theta) / g
# We want to know the relationship between the angle and the velocity to hit a target at distance d
import math 

# For different angles, calculate the required velocity to hit the target at distance d
def calculate_velocity(angle, distance):
    g = 9.81  # Acceleration due to gravity in m/s^2
    theta = math.radians(angle)  # Convert angle to radians
    velocity = math.sqrt((distance * g) / math.sin(2 * theta))  # Calculate velocity using the formula
    return velocity

# There is a limit on x_c, here at x_c, the height of the projectile must be larger than some H
H = 0.2  # Minimum height in meters
x_c = 0.075 # Critical distance in meters

# Function to check if the projectile can clear the height H at distance x_c for a given angle and velocity
def can_clear_height(angle, velocity):
    g = 9.81  # Acceleration due to gravity in m/s^2
    theta = math.radians(angle)  # Convert angle to radians
    y_c = x_c * math.tan(theta) - (g * x_c**2) / (2 * velocity**2 * math.cos(theta)**2)  # Calculate height at distance x_c
    return y_c >= H  # Check if the height is greater than or equal to H
 
if __name__ == "__main__":
    target_distance = 0.27  # Distance to the target in meters

    # Find the minimum angle and corresponding velocity to hit the target while clearing the height at x_c
    for angle in range(1, 90):  # Check angles from 1 to 89 degrees
        velocity = calculate_velocity(angle, target_distance)
        if can_clear_height(angle, velocity):
            print(f"Angle: {angle} degrees, Velocity: {velocity:.2f} m/s")
            break
