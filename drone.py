import heapq

def heuristic(a, b):
    # Manhattan distance
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(grid, start, end):
    # A* algorithm
    frontier = [(0, start)]
    came_from = {}
    cost_so_far = {start: 0}

    while frontier:
        _, current = heapq.heappop(frontier)

        if current == end:
            break

        for next_pos in neighbors(current, grid):
            new_cost = cost_so_far[current] + 1
            if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                cost_so_far[next_pos] = new_cost
                priority = new_cost + heuristic(end, next_pos)
                heapq.heappush(frontier, (priority, next_pos))
                came_from[next_pos] = current

    path = []
    current = end
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()

    return path

def neighbors(pos, grid):
    # Returns all adjacent cells that are not occupied
    row, col = pos
    results = []
    for i in range(-1, 2):
        for j in range(-1, 2):
            if i == 0 and j == 0:
                continue
            neighbor = (row + i, col + j)
            if 0 <= neighbor[0] < len(grid) and 0 <= neighbor[1] < len(grid[0]) and grid[neighbor[0]][neighbor[1]] == 0:
                results.append(neighbor)
    return results

def find_collision(start1, end1, start2, end2, t1, t2):
    # Check if two drones collide
    dx1 = end1[0] - start1[0]
    dy1 = end1[1] - start1[1]
    dx2 = end2[0] - start2[0]
    dy2 = end2[1] - start2[1]
    if dx1 == dx2 and dy1 == dy2:
        # Same path
        return True
    elif t1 == t2:
        # Same starting time
        return start1 == start2
    else:
        # Calculate positions at time of collision
        t = (t1 * dx2 - t2 * dx1) / (dy2 * dx1 - dy1 * dx2)
        if t < 0 or t > 1:
            return False
        s = (t1 * dy2 - t2 * dy1) / (dy2 * dx1 - dy1 * dx2)
        return 0 <= s <= 1

def get_path(start, end, grid):
    # Get the path from start to end using A*
    return astar(grid, start, end)

def get_grid(drones):
    # Initialize a grid with all cells unoccupied
    grid = [[0 for _ in range(20)] for _ in range(20)]
    # Mark cells occupied by drones
    for drone in drones:
        start = (drone[0], drone[1])
        end = (drone[2], drone[3])
        t = drone[4]
        path = get_path(start, end, grid)
        for i, pos in enumerate(path):
            # Check for collisions with other drones
            for j, other_drone in enumerate(drones):
                if j == len(drones) - 1:
                    continue
                if i >= len(other_drone[5]):
                    continue
                other_start = (other_drone[5][i][0], other_drone[5][i][1])
                other_end = (other_drone[5][i][2], other_drone[5][i][3])
                other_t = other_drone[4]
                if find_collision(start, end, other_start, other_end, t, other_t):
                    # Mark cells as occupied by drones
                    grid[pos[0]][pos[1]] = 1
                    break
            else:
                # No collision detected, mark cell as unoccupied
                grid[pos[0]][pos[1]] = 0
        # Add drone's path to its data
        drone.append(path)
    return grid



