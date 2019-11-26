def reward_function(params):

    import math

    # Read input parameters
    distance_from_center = params['distance_from_center']
    progress = params["progress"]
    waypoints = params['waypoints']
    max_waypoints = len(waypoints)
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']
    steering_angle = params['steering_angle']
    is_left_of_center = params['is_left_of_center']
    track_width = params['track_width']
    all_wheels_on_track = params['all_wheels_on_track']
    speed = params['speed']

    # Make the track smaller
    virtual_track_width = track_width * 0.9

    look_ahead_threshold = 3

    # In meter
    MARKER_THRESHOLD_1 = 0.1 * virtual_track_width
    MARKER_THRESHOLD_2 = 0.25 * virtual_track_width
    MARKER_THRESHOLD_3 = 0.7 * virtual_track_width
    
    # In Degree
    DIRECTION_THRESHOLD_1 = 5.0
    DIRECTION_THRESHOLD_2 = 10.0
    DIRECTION_THRESHOLD_3 = 15.0

    reward = 0

    # Get the current track direction
    # Calculate the direction of the center line based on the closest waypoints
    next_point = waypoints[closest_waypoints[1]]
    prev_point = waypoints[closest_waypoints[0]]

    # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
    current_track_direction = math.atan2(
        next_point[1] - prev_point[1], next_point[0] - prev_point[0])
    # Convert to degree
    current_track_direction = math.degrees(current_track_direction)

    # Get the next track direction
    next_closet_waypoints_1 = closest_waypoints[1] + look_ahead_threshold
    next_closet_waypoints_0 = closest_waypoints[0] + look_ahead_threshold
    if closest_waypoints[1] + look_ahead_threshold >= max_waypoints:
        next_closet_waypoints_1 = (closest_waypoints[1] + look_ahead_threshold) - max_waypoints
        next_closet_waypoints_0 = next_closet_waypoints_1 - 1

    next_point = waypoints[next_closet_waypoints_1]
    prev_point = waypoints[next_closet_waypoints_0]
    next_track_direction = math.atan2(
        next_point[1] - prev_point[1], next_point[0] - prev_point[0])
    next_track_direction = math.degrees(next_track_direction)

    if next_track_direction - current_track_direction < 10:
        # Going straight / follow the actual center line

        # 1. Reward if the car is closer to center line and vice versa
        if distance_from_center <= MARKER_THRESHOLD_1:
            reward += 1.5
        elif distance_from_center <= MARKER_THRESHOLD_2:
            reward += 0.25
        elif distance_from_center <= MARKER_THRESHOLD_3:
            reward += 0.1
        else:
            return 1e-3  # likely crashed/ close to off track

        # 2. Reward if the car heading angle difference is small
        direction_diff = abs(current_track_direction - heading)
        if direction_diff > 180:
            direction_diff = 360 - direction_diff

        if direction_diff < DIRECTION_THRESHOLD_1:
            reward += 3.0
        elif direction_diff < DIRECTION_THRESHOLD_2:
            reward += 1.5
        elif direction_diff < DIRECTION_THRESHOLD_3:
            reward += 0.1
        # else:
        #     return 1e-3

        # 3. Reward if the steering angle is small
        # if abs(steering_angle) < DIRECTION_THRESHOLD_1:
        #     reward += 1.5
        # elif abs(steering_angle) < DIRECTION_THRESHOLD_2:
        #     reward += 0.25
        # elif abs(steering_angle) < DIRECTION_THRESHOLD_3:
        #     reward += 0.1
        # else:
        #     return 1e-3

    else:
        # Turning / follow the virtual center
        
        # Let start with fixed offset
        center_offset = track_width / 2 / 3

        if next_track_direction - current_track_direction > 0:
            # Going left

            # Reward if it is steering left, not too much reward,
            # make sure not over steering
            if steering_angle < 0:
                reward += 2

            # Reward if staying near virtual center
            if is_left_of_center:
                distance_from_virtual_center = abs(distance_from_center - center_offset)
            else:
                distance_from_virtual_center = distance_from_center + center_offset
                
            if distance_from_virtual_center <= MARKER_THRESHOLD_1:
                reward += 2
            elif distance_from_virtual_center <= MARKER_THRESHOLD_2:
                reward += 1
            elif distance_from_virtual_center <= MARKER_THRESHOLD_3:
                reward += 0.5

        else:
            # Going right

            # Reward if it is steering right, not too much reward,
            # make sure not over steering
            if steering_angle > 0:
                reward += 2

            # Reward if staying near virtual center
            if not is_left_of_center:
                distance_from_virtual_center = abs(distance_from_center - center_offset)
            else:
                distance_from_virtual_center = distance_from_center + center_offset
                
            if distance_from_virtual_center <= MARKER_THRESHOLD_1:
                reward += 2
            elif distance_from_virtual_center <= MARKER_THRESHOLD_2:
                reward += 1
            elif distance_from_virtual_center <= MARKER_THRESHOLD_3:
                reward += 0.5

    # Set the speed threshold based your action space 
    # SPEED_THRESHOLD = 1.0 

    # if not all_wheels_on_track:
    #     # Penalize if the car goes off track
    #     return 1e-3
    # elif speed < SPEED_THRESHOLD:
    #     # Penalize if the car goes too slow
    #     reward += 0.2
    # else:
    #     # High reward if the car stays on track and goes fast
    #     reward += 1.0

    # Making sure getting to finish line
    reward = reward + (progress/10)

    return float(reward)
