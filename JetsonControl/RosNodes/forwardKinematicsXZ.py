"""
Arm Kinematics Module for Robotic Arm Control

This module provides forward and inverse kinematics calculations for a 3-DOF robotic arm,
along with conversion functions between joint angles and potentiometer values.

Usage:
    from forwardKinematicsXZ import get_potentiometer_values_for_position
    
    # Get potentiometer values for a target position
    pot_values = get_potentiometer_values_for_position(target_x, target_z)
    if pot_values:
        print(f"Actuator 1: {pot_values['actuator1']}")
        print(f"Actuator 2: {pot_values['actuator2']}")
        print(f"Actuator 3: {pot_values['actuator3']}")
"""


import numpy as np
import math

# Segment lengths in cm
L1 = 49.5
L2 = 49.5
L3 = 49.5

# Potentiometer ranges for each actuator
POTENTIOMETER_RANGES = {
    'actuator1': {'min': 37, 'max': 790},
    'actuator2': {'min': 37, 'max': 952},
    'actuator3': {'min': 29, 'max': 957}
}

# Angle ranges for each theta (in degrees)
ANGLE_RANGES = {
    'theta1': {'min': 75, 'max': 135},
    'theta2': {'min': 70, 'max': 97},
    'theta3': {'min': 90, 'max': 154}
}

def angles_to_potentiometer_values(theta1, theta2, theta3):
    """
    Convert joint angles to potentiometer values for each actuator.
    
    Args:
        theta1, theta2, theta3: Joint angles in degrees
        
    Returns:
        dict: Dictionary with potentiometer values for each actuator
    """
    potentiometer_values = {}
    
    # Map theta1 to actuator1 potentiometer
    theta1_normalized = (theta1 - ANGLE_RANGES['theta1']['min']) / (ANGLE_RANGES['theta1']['max'] - ANGLE_RANGES['theta1']['min'])
    pot1_value = POTENTIOMETER_RANGES['actuator1']['min'] + theta1_normalized * (POTENTIOMETER_RANGES['actuator1']['max'] - POTENTIOMETER_RANGES['actuator1']['min'])
    potentiometer_values['actuator1'] = int(round(pot1_value))
    
    # Map theta2 to actuator2 potentiometer
    theta2_normalized = (theta2 - ANGLE_RANGES['theta2']['min']) / (ANGLE_RANGES['theta2']['max'] - ANGLE_RANGES['theta2']['min'])
    pot2_value = POTENTIOMETER_RANGES['actuator2']['min'] + theta2_normalized * (POTENTIOMETER_RANGES['actuator2']['max'] - POTENTIOMETER_RANGES['actuator2']['min'])
    potentiometer_values['actuator2'] = int(round(pot2_value))
    
    # Map theta3 to actuator3 potentiometer
    theta3_normalized = (theta3 - ANGLE_RANGES['theta3']['min']) / (ANGLE_RANGES['theta3']['max'] - ANGLE_RANGES['theta3']['min'])
    pot3_value = POTENTIOMETER_RANGES['actuator3']['min'] + theta3_normalized * (POTENTIOMETER_RANGES['actuator3']['max'] - POTENTIOMETER_RANGES['actuator3']['min'])
    potentiometer_values['actuator3'] = int(round(pot3_value))
    
    return potentiometer_values

def potentiometer_values_to_angles(pot1, pot2, pot3):
    """
    Convert potentiometer values back to joint angles.
    
    Args:
        pot1, pot2, pot3: Potentiometer values for actuators 1, 2, 3
        
    Returns:
        tuple: (theta1, theta2, theta3) in degrees
    """
    # Convert actuator1 potentiometer to theta1
    pot1_normalized = (pot1 - POTENTIOMETER_RANGES['actuator1']['min']) / (POTENTIOMETER_RANGES['actuator1']['max'] - POTENTIOMETER_RANGES['actuator1']['min'])
    theta1 = ANGLE_RANGES['theta1']['min'] + pot1_normalized * (ANGLE_RANGES['theta1']['max'] - ANGLE_RANGES['theta1']['min'])
    
    # Convert actuator2 potentiometer to theta2
    pot2_normalized = (pot2 - POTENTIOMETER_RANGES['actuator2']['min']) / (POTENTIOMETER_RANGES['actuator2']['max'] - POTENTIOMETER_RANGES['actuator2']['min'])
    theta2 = ANGLE_RANGES['theta2']['min'] + pot2_normalized * (ANGLE_RANGES['theta2']['max'] - ANGLE_RANGES['theta2']['min'])
    
    # Convert actuator3 potentiometer to theta3
    pot3_normalized = (pot3 - POTENTIOMETER_RANGES['actuator3']['min']) / (POTENTIOMETER_RANGES['actuator3']['max'] - POTENTIOMETER_RANGES['actuator3']['min'])
    theta3 = ANGLE_RANGES['theta3']['min'] + pot3_normalized * (ANGLE_RANGES['theta3']['max'] - ANGLE_RANGES['theta3']['min'])
    
    return theta1, theta2, theta3

def corrected_forward_kinematics(theta1_deg, theta2_deg, theta3_deg):

    # Convert degrees to radians
    theta1 = np.radians(theta1_deg)
    theta2 = np.radians(theta2_deg)
    theta3 = np.radians(theta3_deg)

    # Calculate absolute joint orientations
    joint1_angle = theta1
    joint2_angle = joint1_angle + (np.pi - theta2)
    joint3_angle = joint2_angle + (np.pi - theta3)

    # Positions
    x0, z0 = 0, 0
    x1 = x0 + L1 * np.cos(joint1_angle)
    z1 = z0 + L1 * np.sin(joint1_angle)
    
    x2 = x1 + L2 * np.cos(joint2_angle)
    z2 = z1 + L2 * np.sin(joint2_angle)
    
    x3 = x2 + L3 * np.cos(joint3_angle)
    z3 = z2 + L3 * np.sin(joint3_angle)
    
    return [(x0, z0), (x1, z1), (x2, z2), (x3, z3)], (x3, z3)

def inverse_kinematics_search(target_x, target_z, theta1_min=75, theta1_max=135, 
                             theta2_min=70, theta2_max=97, theta3_min=90, theta3_max=154,
                             step=1):

    # Pregătește gridul de unghiuri
    theta1_vals = np.arange(theta1_min, theta1_max + 1, step, dtype=np.float32)
    theta2_vals = np.arange(theta2_min, theta2_max + 1, step, dtype=np.float32)
    theta3_vals = np.arange(theta3_min, theta3_max + 1, step, dtype=np.float32)

    n1, n2, n3 = len(theta1_vals), len(theta2_vals), len(theta3_vals)
    total = n1 * n2 * n3

    combos = np.zeros((total, 3), dtype=np.float32)
    idx = 0
    for i in range(n1):
        for j in range(n2):
            for k in range(n3):
                combos[idx, 0] = theta1_vals[i]
                combos[idx, 1] = theta2_vals[j]
                combos[idx, 2] = theta3_vals[k]
                idx += 1

    distances = np.zeros(total, dtype=np.float32)
    xs = np.zeros(total, dtype=np.float32)
    zs = np.zeros(total, dtype=np.float32)

    try:
        import pycuda.autoinit
        import pycuda.driver as drv
        import pycuda.gpuarray as gpuarray
        from pycuda.compiler import SourceModule

        kernel_code = """
        __global__ void kinematics(float *combos, float target_x, float target_z, float *distances, float *xs, float *zs, float L1, float L2, float L3, int total) {
            int i = blockIdx.x * blockDim.x + threadIdx.x;
            if (i < total) {
                float t1 = combos[3*i+0] * 0.01745329252f; // deg to rad
                float t2 = combos[3*i+1] * 0.01745329252f;
                float t3 = combos[3*i+2] * 0.01745329252f;
                float j1 = t1;
                float j2 = j1 + (3.14159265359f - t2);
                float j3 = j2 + (3.14159265359f - t3);
                float x1 = L1 * cosf(j1);
                float z1 = L1 * sinf(j1);
                float x2 = x1 + L2 * cosf(j2);
                float z2 = z1 + L2 * sinf(j2);
                float x3 = x2 + L3 * cosf(j3);
                float z3 = z2 + L3 * sinf(j3);
                xs[i] = x3;
                zs[i] = z3;
                float dx = x3 - target_x;
                float dz = z3 - target_z;
                distances[i] = sqrtf(dx*dx + dz*dz);
            }
        }
        """
        mod = SourceModule(kernel_code)
        kinematics = mod.get_function("kinematics")

        combos_gpu = gpuarray.to_gpu(combos.flatten())
        distances_gpu = gpuarray.zeros(total, np.float32)
        xs_gpu = gpuarray.zeros(total, np.float32)
        zs_gpu = gpuarray.zeros(total, np.float32)

        block_size = 256
        grid_size = (total + block_size - 1) // block_size

        kinematics(
            combos_gpu,
            np.float32(target_x),
            np.float32(target_z),
            distances_gpu,
            xs_gpu,
            zs_gpu,
            np.float32(L1),
            np.float32(L2),
            np.float32(L3),
            np.int32(total),
            block=(block_size,1,1), grid=(grid_size,1)
        )

        distances = distances_gpu.get()
        xs = xs_gpu.get()
        zs = zs_gpu.get()

    except Exception as e:
        # Fallback CPU (NumPy vectorized)
        t1 = np.radians(combos[:, 0])
        t2 = np.radians(combos[:, 1])
        t3 = np.radians(combos[:, 2])
        j1 = t1
        j2 = j1 + (np.pi - t2)
        j3 = j2 + (np.pi - t3)
        x1 = L1 * np.cos(j1)
        z1 = L1 * np.sin(j1)
        x2 = x1 + L2 * np.cos(j2)
        z2 = z1 + L2 * np.sin(j2)
        x3 = x2 + L3 * np.cos(j3)
        z3 = z2 + L3 * np.sin(j3)
        xs = x3
        zs = z3
        distances = np.sqrt((xs - target_x)**2 + (zs - target_z)**2)

    min_idx = np.argmin(distances)
    best_theta1, best_theta2, best_theta3 = combos[min_idx]
    best_x = xs[min_idx]
    best_z = zs[min_idx]
    best_distance = distances[min_idx]
    return (float(best_theta1), float(best_theta2), float(best_theta3), float(best_x), float(best_z), float(best_distance))

def get_potentiometer_values_for_position(target_x, target_z, search_step=1):
    """
    Main function to get potentiometer values for a desired end-effector position.
    
    Args:
        target_x (float): Target X coordinate in cm
        target_z (float): Target Z coordinate in cm
        search_step (int): Search resolution in degrees (default: 1)
    
    Returns:
        dict or None: Dictionary with potentiometer values for each actuator, or None if no solution found
        {
            'actuator1': int,
            'actuator2': int, 
            'actuator3': int,
            'achieved_x': float,
            'achieved_z': float,
            'error': float
        }
    """
    # Find the best angles for the target position
    result = inverse_kinematics_search(target_x, target_z, step=search_step)
    
    if result is None:
        return None
    
    theta1, theta2, theta3, achieved_x, achieved_z, error = result
    
    # Convert angles to potentiometer values
    pot_values = angles_to_potentiometer_values(theta1, theta2, theta3)
    
    # Add achieved position and error to the result
    pot_values['achieved_x'] = achieved_x
    pot_values['achieved_z'] = achieved_z
    pot_values['error'] = error
    
    return pot_values

# Example usage (can be run directly for testing)
if __name__ == "__main__":
    # Test the main function
    test_x, test_z = -50.0, -10.0
    
    print(f"Căutare valori potentiometru pentru poziția: x={test_x:.1f} cm, z={test_z:.1f} cm")
    result = get_potentiometer_values_for_position(test_x, test_z)
    
    if result:
        print(f"Soluție găsită:")
        print(f"  Actuator 1: {result['actuator1']}")
        print(f"  Actuator 2: {result['actuator2']}")
        print(f"  Actuator 3: {result['actuator3']}")
        print(f"Poziția obținută: x={result['achieved_x']:.2f} cm, z={result['achieved_z']:.2f} cm")
        print(f"Eroarea: {result['error']:.4f} cm")
    else:
        print("Nu s-a găsit o soluție validă pentru poziția cerută.")