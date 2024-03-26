# https://github.com/FazilMammadli/ENPM-661--Project-3/tree/main

from queue import PriorityQueue
import time
import numpy as np
import cv2
# from google.colab.patches import cv2_imshow --> Uncomment this If you are using Google Colab

class PriorityNodeQueue:
    def __init__(self):
        self.nodes_queue = PriorityQueue()
        # Maps nodes to their corresponding priorities and queue positions
        self.node_tracker = {}

    # Method to insert or update a node's priority in the queue
    def insert_or_update(self, node_priority, node):
        if node in self.node_tracker:
            existing_priority, _ = self.node_tracker[node]
            if node_priority < existing_priority:
                # Adjust the node's priority details
                self.node_tracker[node] = (node_priority, node)
                # Re-insert with updated priority
                self.nodes_queue.put((node_priority, node))
        else:
            node_entry = (node_priority, node)
            self.node_tracker[node] = node_entry
            self.nodes_queue.put(node_entry)

    def retrieve_next(self):
        node_priority, node = self.nodes_queue.get()
        # Validate and clean up if node is still tracked
        if node in self.node_tracker:
            self.node_tracker.pop(node)
        return node_priority, node

    def is_empty(self):
        return self.nodes_queue.empty()

    def remove_node(self, node):
        if node in self.node_tracker:
            node_entry = self.node_tracker.pop(node)
            return node_entry[1]
        return None

    # Check if a node is present in the queue
    def __contains__(self, node):
        return node in self.node_tracker



def is_node_in_obstacle(candidate_node, obstacle_clearance):
    # Redefining variables for clarity and modifying the approach slightly
    x, y = candidate_node[:2]
    is_obstructed = False

    # Adjust obstacle boundaries considering clearance
    hex_slope1 = ((400 + obstacle_clearance) - (325 + obstacle_clearance)) / (650 - (520 - obstacle_clearance))
    hex_slope2 = ((325 + obstacle_clearance) - (400 + obstacle_clearance)) / ((780 + obstacle_clearance) - 650)
    hex_slope3 = ((175 - obstacle_clearance) - (100 - obstacle_clearance)) / ((780 + obstacle_clearance) - 650)
    hex_slope4 = ((100 - obstacle_clearance) - (175 - obstacle_clearance)) / (650 - (520 - obstacle_clearance))

    # Map boundaries check
    if x < 0 + obstacle_clearance or x > 1200 - obstacle_clearance or y < 0 + obstacle_clearance or y > 500 - obstacle_clearance:
        is_obstructed = True

    # Rectangle obstacles
    elif 100 - obstacle_clearance < x < 175 + obstacle_clearance and y > 100 - obstacle_clearance:
        is_obstructed = True
    elif 275 - obstacle_clearance < x < 350 + obstacle_clearance and y < 400 + obstacle_clearance:
        is_obstructed = True

    # Hexagon obstacle
    elif 520 - obstacle_clearance < x < 780 + obstacle_clearance and (hex_slope1 * (x - (520 - obstacle_clearance)) - (y - (325 + obstacle_clearance)) > 0) and (hex_slope2 * (x - 650) - (y - (400 + obstacle_clearance)) > 0) and (hex_slope3 * (x - 650) - (y - (100 - obstacle_clearance)) < 0) and (hex_slope4 * (x - (520 - obstacle_clearance)) - (y - (175 - obstacle_clearance)) < 0):
        is_obstructed = True

    # Additional obstacles
    elif 900 - obstacle_clearance < x < 1020 + obstacle_clearance and 375 - obstacle_clearance < y < 450 + obstacle_clearance:
        is_obstructed = True
    elif 900 - obstacle_clearance < x < 1020 + obstacle_clearance and 50 - obstacle_clearance < y < 125 + obstacle_clearance:
        is_obstructed = True
    elif 1020 - obstacle_clearance < x < 1100 + obstacle_clearance and 50 - obstacle_clearance < y < 450 + obstacle_clearance:
        is_obstructed = True

    return is_obstructed

def has_reached_goal(start_node, goal_node):
    # Extracting node and goal coordinates
    x_node, y_node, theta_node = start_node
    x_goal, y_goal, theta_goal = goal_node

    # Calculating the Euclidean distance between the current node and the goal node
    distance = np.sqrt((x_node - x_goal)**2 + (y_node - y_goal)**2)

    # Checking if the node is within a specified radius from the goal and matches the desired orientation
    is_within_goal_radius = distance < 3
    has_matching_orientation = theta_node == theta_goal

    # The goal is reached if the node is within the goal radius and has the matching orientation
    return is_within_goal_radius and has_matching_orientation



# Adapted for generating hexagon coordinates with clearance adjustment
def calculate_hexagon_points(centre, side_length, rotation_angle, safety_clearance):
    # Hexagon points calculation
    hex_angles = np.radians(np.arange(0, 360, 60) + rotation_angle)
    x_pts = (side_length + safety_clearance) * np.cos(hex_angles) + centre[0]
    y_pts = (side_length + safety_clearance) * np.sin(hex_angles) + centre[1]

    hex_points = [(int(x), int(y)) for x, y in zip(x_pts, y_pts)]
    return hex_points

def calculate_square_points(node_position):
    # Extracting node information: center and orientation in radians
    center_x = node_position[0]
    center_y = node_position[1]
    orientation_degrees = node_position[2] * 30  # Convert to degrees if needed
    orientation_radians = np.radians(orientation_degrees)  # Convert degrees to radians for calculation

    # Side length of the square
    side_length = 5.0

    # Calculate coordinates for each corner of the square based on orientation and position
    # Adjusting for graphical Y-axis inversion directly in calculation
    top_right_x = center_x + side_length * (np.cos(orientation_radians) - np.sin(orientation_radians))
    top_right_y = 500 - (center_y + side_length * (np.sin(orientation_radians) + np.cos(orientation_radians)))

    top_left_x = center_x + side_length * (-np.cos(orientation_radians) - np.sin(orientation_radians))
    top_left_y = 500 - (center_y + side_length * (-np.sin(orientation_radians) + np.cos(orientation_radians)))

    bottom_left_x = center_x + side_length * (-np.cos(orientation_radians) + np.sin(orientation_radians))
    bottom_left_y = 500 - (center_y + side_length * (-np.sin(orientation_radians) - np.cos(orientation_radians)))

    bottom_right_x = center_x + side_length * (np.cos(orientation_radians) + np.sin(orientation_radians))
    bottom_right_y = 500 - (center_y + side_length * (np.sin(orientation_radians) - np.cos(orientation_radians)))

    # Assemble coordinates in a numpy array for easy manipulation or plotting
    square_coords = np.array([
        [top_right_x, top_right_y],
        [top_left_x, top_left_y],
        [bottom_left_x, bottom_left_y],
        [bottom_right_x, bottom_right_y]
    ])

    return square_coords

def visualize_start_and_goal_nodes(canvas, start_node, goal_node):
    # Drawing the start node as a green square
    cv2.circle(canvas, (start_node[0], 500 - start_node[1]), radius=5, color=(0, 0, 255), thickness=cv2.FILLED)
    # Drawing the goal node as a red square
    cv2.circle(canvas, (goal_node[0], 500 - goal_node[1]), radius=5, color=(0, 255, 0), thickness=cv2.FILLED)
def visualize_obstacles(canvas, obstacle_shapes, video_writer):
    for shape in obstacle_shapes:
        # For rectangular obstacles
        if len(shape) == 2 and shape != ((0, 0), (1200, 500)):
            # Adjusting coordinates for CV2 drawing
            start_point = (shape[0][0], 500 - shape[0][1])
            end_point = (shape[1][0], 500 - shape[1][1])
            cv2.rectangle(canvas, pt1=start_point, pt2=end_point, color=(255, 255, 255), thickness=-1)
        # For hexagonal or complex polygonal obstacles
        elif len(shape) == 6:
            polygon_points = np.array(shape, dtype=np.int32).reshape((-1, 1, 2))
            cv2.fillPoly(canvas, [polygon_points], color=(255, 255, 255))

    # Display the updated canvas and write to the video
    # cv2_imshow(canvas)
    video_writer.write(canvas)
    cv2.waitKey(2000)  # Delay to allow visualization to be seen



def visualize_explored_nodes(canvas, explored_points, movement_step, video_writer):
    # Iterate over each point to draw lines showing exploration
    for i, point in enumerate(explored_points[1:], start=1):  # Skip the first point
        # Extract the node's position and orientation
        x, y, orientation = point

        # Convert orientation to degrees and adjust for drawing the line backwards
        angle_degrees = (orientation * 30 + 180) % 360
        angle_radians = np.radians(angle_degrees)  # Convert to radians for trigonometric functions

        # Calculate the end point of the line based on the step size
        x_endpoint = int(x + (2 * movement_step * np.cos(angle_radians)))
        y_endpoint = int(y + (2 * movement_step * np.sin(angle_radians)))

        # Adjust y coordinates for visualization (500 - y) because the origin (0,0) is at the top-left corner
        cv2.line(canvas, (x, 500 - y), (x_endpoint, 500 - y_endpoint), (200, 0, 0), thickness=1)

        # Update the visualization after every 1000 steps to improve performance
        if i % 1000 == 0:
            video_writer.write(canvas)
            cv2.waitKey(int(3000 / 120))  # Introduce a slight delay for visualization

    # Ensure the final state is visualized
    video_writer.write(canvas)

def visualize_optimal_path(canvas, optimal_path, movement_step, video_writer):
    print(f"Visualizing path with {len(optimal_path)} steps.")
    
    # Draw each segment of the path
    for idx in range(len(optimal_path) - 1):
        current_point = optimal_path[idx]
        next_point = optimal_path[idx + 1]

        # Draw the path line as a yellow straight line
        cv2.line(canvas, (current_point[0], 500 - current_point[1]), (next_point[0], 500 - next_point[1]), (0, 255, 255), thickness=2)

    # Save the final visualization
    video_writer.write(canvas)
    # cv2_imshow(canvas)
    cv2.waitKey(1000)  # Wait a bit to show the last frame


def append_blank_frames(canvas, video_writer, frame_rate, duration_seconds):
    total_blank_frames = frame_rate * duration_seconds
    for _ in range(total_blank_frames):
        video_writer.write(canvas)

def generate_animation(obstacle_shapes, explored_nodes, optimal_path, start_node, goal_node, step_size):
    # Video configuration
    video_codec = cv2.VideoWriter_fourcc(*'mp4v')
    frames_per_second = 60
    output_video = cv2.VideoWriter('a_star_path_finding.mp4', video_codec, frames_per_second, (1200, 500))

    # Create a white canvas
    canvas = np.full((500, 1200, 3), 0, dtype=np.uint8)

    # Drawing static elements: start and goal nodes, and obstacles
    visualize_start_and_goal_nodes(canvas, start_node, goal_node)
    visualize_obstacles(canvas, obstacle_shapes, output_video)
    
    # Adding an initial pause
    append_blank_frames(canvas, output_video, frames_per_second, 2)
    
    # Visualizing the exploration and the optimal path
    visualize_explored_nodes(canvas, explored_nodes, step_size, output_video)
    
    # Redrawing static elements to ensure they are visible over the explored nodes
    visualize_start_and_goal_nodes(canvas, start_node, goal_node)
    
    visualize_optimal_path(canvas, optimal_path, step_size, output_video)
    
    # Adding a final pause
    append_blank_frames(canvas, output_video, frames_per_second, 2)

    # Cleanup
    output_video.release()
    cv2.destroyAllWindows()  # Ensure all OpenCV windows are closed

def define_obstacle_space():
    obstacles = []

    # The entire map as a boundary obstacle
    obstacles.append(((0, 0), (1200, 500)))
    # Rectangular obstacles
    obstacles.append(((100, 100), (175, 500)))
    obstacles.append(((275, 0), (350, 400)))
    # Additional rectangular obstacles
    obstacles.append(((900, 50), (1100, 125)))
    obstacles.append(((900, 375), (1100, 450)))
    obstacles.append(((1020, 125), (1100, 375)))

    # Adding a hexagon obstacle using the calculate_hexagon_points method
    hexagon_points = calculate_hexagon_points((650, 250), 150, 90, 0)
    obstacles.append(hexagon_points)

    return obstacles

def generate_nodes(node_state, step_size, clearance):
    # Unpack the current node state
    x, y, orientation = node_state
    # Adjust x and y for calculation (assuming original coordinates are scaled up by 2)
    x_scaled = x / 2
    y_scaled = y / 2

    new_nodes = []

    # Generate new node positions for each of the potential 5 moves
    for delta_orientation in range(-2, 3):  # From -60 to +60 degrees in 30-degree increments
        # Calculate the new orientation (ensuring it wraps correctly around the 360-degree boundary)
        new_orientation = (orientation + delta_orientation) % 12

        # Convert the new orientation to radians for trigonometry
        angle_rad = np.radians(new_orientation * 30)

        # Calculate the new x and y based on the move in the direction of the new orientation
        new_x_scaled = x_scaled + step_size * np.cos(angle_rad)
        new_y_scaled = y_scaled + step_size * np.sin(angle_rad)

        # Scale back up by 2 to match the coordinate system of the grid
        new_x = round(new_x_scaled * 2)
        new_y = round(new_y_scaled * 2)

        # Ensure the new node is within the bounds and not in an obstacle
        if not (new_x < 5 + clearance or new_x > 1195 - clearance or new_y < 5 + clearance or new_y > 495 - clearance):
            # Append the new node with the cost to come (c2c), coordinates, and orientation
            new_nodes.append((step_size * 2, (new_x, new_y, new_orientation)))

    return new_nodes


def calculate_heuristic(current_node, goal_node, weight):
    # Use Euclidean distance as the heuristic
    distance = np.sqrt((goal_node[0] - current_node[0])**2 + (goal_node[1] - current_node[1])**2)
    return weight * distance

def a_star_search(start, goal, heuristic_weight, step_size, obstacle_clearance):
    # Scale start and goal nodes to match the grid
    start_node = (int(start[0] * 2), int(start[1] * 2), start[2])
    goal_node = (int(goal[0] * 2), int(goal[1] * 2), goal[2])

    # Initialize the cost grid with infinity
    cost_grid = [[[float('inf')] * 12 for _ in range(500)] for _ in range(1200)]
    cost_grid[start_node[0]][start_node[1]][start_node[2]] = 0

    # Grid to store the parent of each node for path reconstruction
    parent_grid = [[[None] * 12 for _ in range(500)] for _ in range(1200)]

    # Tracking visited nodes to prevent re-visiting
    visited_grid = [[[False] * 12 for _ in range(500)] for _ in range(1200)]
    visited_nodes = []

    # Initialize priority queue with the start node
    open_nodes = PriorityNodeQueue()
    open_nodes.insert_or_update(0, start_node)

    while not open_nodes.is_empty():
        _, current_node = open_nodes.retrieve_next()
        visited_grid[current_node[0]][current_node[1]][current_node[2]] = True
        visited_nodes.append(current_node)

        # Check for goal condition
        if has_reached_goal(current_node, goal_node):
            return parent_grid, visited_nodes, "Goal reached successfully."

        # Generate potential moves
        for cost_to_come, neighbor in generate_nodes(current_node, step_size, obstacle_clearance):
            # Avoid revisiting and ensure the neighbor is not in an obstacle
            if not visited_grid[neighbor[0]][neighbor[1]][neighbor[2]] and not is_node_in_obstacle(neighbor, obstacle_clearance):
                new_cost = cost_grid[current_node[0]][current_node[1]][current_node[2]] + cost_to_come
                # Update costs if a better path is found
                if new_cost < cost_grid[neighbor[0]][neighbor[1]][neighbor[2]]:
                    cost_grid[neighbor[0]][neighbor[1]][neighbor[2]] = new_cost
                    priority = new_cost + calculate_heuristic(neighbor, goal_node, heuristic_weight)
                    open_nodes.insert_or_update(priority, neighbor)
                    parent_grid[neighbor[0]][neighbor[1]][neighbor[2]] = current_node

    # If the loop ends without reaching the goal, return a failure message along with the visited nodes and the parent grid.
    return parent_grid, visited_nodes, "Failed to find a path to the goal."



def reconstruct_path(ancestry_record, discovery_log, initiation_point):
    """
    Traces back the optimal path from the goal node to the start node using the ancestry record.
    
    Args:
    - ancestry_record: A 3D array mapping each node to its parent.
    - discovery_log: Sequence of nodes visited during the search.
    - initiation_point: Starting node as a tuple (x, y, orientation_index).

    Returns:
    - optimal_route: Sequence of nodes forming the optimal path from start to goal.
    """
    # Initialize the path with the goal node, assuming the last discovered node is the goal
    terminal_node = discovery_log[-1]
    optimal_route = [terminal_node]

    # Loop until the start node is reached
    while initiation_point != terminal_node:
        # Calculate node's coordinates in the ancestry record
        node_x, node_y, node_orientation = terminal_node
        if ancestry_record[node_x][node_y][node_orientation] is None:
            # Break if no parent found, indicating an issue
            print(f"No parent found for node: {terminal_node}. Path reconstruction may be incomplete.")
            break
        # Update the current node to its parent node
        terminal_node = ancestry_record[node_x][node_y][node_orientation]
        # Prepend the parent node to the optimal route
        optimal_route.insert(0, terminal_node)

    return optimal_route



# Main

def main():
    # Configuring the environment for path finding.
    obstacles = define_obstacle_space()
    h_weight = 1  # Adjusting the influence of heuristic.

    clearance = int(input('Enter clearance around obstacles: '))
    s_size = int(input('Enter step size increment (1-10): '))

    # Prompting and validating the starting position.
    start_x = int(input('Start X (0-1200): ')) // 2
    start_y = int(input('Start Y (0-500): ')) // 2
    start_orientation = int(input('Start orientation (0-360, in multiples of 30): ')) // 30
    start_position = tuple((start_x * 2, start_y * 2, start_orientation))
    while is_node_in_obstacle(start_position, clearance):
        print('Invalid start position. Re-enter values.')
        start_x = int(input('Start X (0-1200): ')) // 2
        start_y = int(input('Start Y (0-500): ')) // 2
        start_orientation = int(input('Start orientation (0-360, in multiples of 30): ')) // 30
        start_position = tuple((start_x * 2, start_y * 2, start_orientation))
    start_position = tuple((start_x , start_y , start_orientation))
    # Prompting and validating the goal position.
    goal_x = int(input('Goal X (0-1200): ')) // 2
    goal_y = int(input('Goal Y (0-500): ')) // 2
    goal_orientation = int(input('Goal orientation (0-360, in multiples of 30): ')) // 30
    goal_position = tuple((goal_x * 2, goal_y * 2, goal_orientation))
    while is_node_in_obstacle(goal_position, clearance):
        print('Invalid goal position. Re-enter values.')
        goal_x = int(input('Goal X (0-1200): ')) // 2
        goal_y = int(input('Goal Y (0-500): ')) // 2
        goal_orientation = int(input('Goal orientation (0-360, in multiples of 30): ')) // 30
        goal_position = (goal_x * 2, goal_y * 2, goal_orientation)
    goal_position = tuple((goal_x , goal_y , goal_orientation))

    start = (int(start_position[0]*2), int(start_position[1]*2), start_position[2])
    goal = (int(goal_position[0]*2), int(goal_position[1]*2), goal_position[2])
    # Timing the search process.
    start_time = time.time()

    print('Commencing node exploration...')
    search_results = a_star_search(start_position, goal_position, h_weight, s_size, clearance)
    parent_grid = search_results[0]
    visited_list = search_results[1]

    print('Path calculation in progress...')
    route = reconstruct_path(parent_grid, visited_list, start)

    # Reporting the duration of the search.
    finish_time = time.time()
    print(f'Duration of path discovery: {finish_time - start_time:.2f} sec.')

    # Visualizing the path and exploration.
    generate_animation(obstacles, visited_list, route, start, goal, s_size)

if __name__ == "__main__":
    main()


