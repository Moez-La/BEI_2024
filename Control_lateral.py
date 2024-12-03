

def lateral_control(self):
        """
        Lateral control to calculate steering angle based on the trajectory.
        """
        if not self.path or not self.pos_x_temp or not self.pos_y_temp:
            return 0

        # Current position of the vehicle
        current_pos = np.array([self.pos_x_temp[-1], self.pos_y_temp[-1]])

        # Find the closest point on the trajectory
        distances = [np.linalg.norm(current_pos - np.array(p)) for p in self.path]
        closest_idx = np.argmin(distances)

        # Select the next target point
        next_idx = (closest_idx + 1) % len(self.path)
        target_point = np.array(self.path[next_idx])

        # Calculate the required steering angle
        path_vector = target_point - current_pos
        desired_angle = np.arctan2(path_vector[1], path_vector[0])

        # Angular error
        current_angle = self.theta_temp[-1]
        angle_error = desired_angle - current_angle
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))  # Normalize to [-pi, pi]

        # Proportional control for steering angle
        k_p = 2.0
        steering_angle = k_p * angle_error
        return steering_angle