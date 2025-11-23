import numpy as np
import matplotlib.pyplot as plt
import random

GRID_SIZE = 50
NUM_ROBOTS = 10
NUM_FOOD_SOURCES = 3
MAX_STEPS = 5001
PHEROMONE_DECAY = 0.95          # Evaporation
PHEROMONE_DECAY_PERTURBED = 0.9 # Perturbation 
PHEROMONE_DEPOSIT = 5.0

pheromone_grid = np.zeros((GRID_SIZE, GRID_SIZE))
nest_position = (5, GRID_SIZE // 2)
food_positions = []

# Randomly generate 3 food sources far from nest
for _ in range(NUM_FOOD_SOURCES):
    fx = random.randint(GRID_SIZE // 2 + 5, GRID_SIZE - 5)
    fy = random.randint(5, GRID_SIZE - 5)
    food_positions.append((fx, fy))

def random_start():
    return [nest_position[0], nest_position[1]]

def move_toward(pos, target):
    dx = np.sign(target[0] - pos[0])
    dy = np.sign(target[1] - pos[1])
    return [int(pos[0] + dx), int(pos[1] + dy)]

def bias_random_walk(pos, gradient):
    dx, dy = gradient
    if random.random() < 0.9:
        pos[0] = max(0, min(GRID_SIZE-1, int(pos[0] + np.sign(dx))))
        pos[1] = max(0, min(GRID_SIZE-1, int(pos[1] + np.sign(dy))))
    else:
        pos[0] = max(0, min(GRID_SIZE-1, int(pos[0] + random.choice([-1, 0, 1]))))
        pos[1] = max(0, min(GRID_SIZE-1, int(pos[1] + random.choice([-1, 0, 1]))))
    return pos

def get_gradient(pos):
    x, y = int(pos[0]), int(pos[1])
    gx = pheromone_grid[min(GRID_SIZE-1, x+1), y] - pheromone_grid[max(0, x-1), y]
    gy = pheromone_grid[x, min(GRID_SIZE-1, y+1)] - pheromone_grid[x, max(0, y-1)]
    return (gx, gy)

def detect_food(pos):
    return tuple(pos) in food_positions

def at_nest(pos):
    return tuple(pos) == nest_position

class ForagingRobot:
    def __init__(self):
        self.state = "exploring"
        self.position = random_start()
        self.trail_strength = 0

    def step(self):
        x, y = int(self.position[0]), int(self.position[1])

        if self.state == "exploring":
            if detect_food(self.position):
                self.state = "returning"
                self.trail_strength = PHEROMONE_DEPOSIT
            else:
                grad = get_gradient(self.position)
                self.position = bias_random_walk(self.position, grad)

        elif self.state == "returning":
            pheromone_grid[x, y] += self.trail_strength
            self.trail_strength *= 0.99
            self.position = move_toward(self.position, nest_position)
            if at_nest(self.position):
                self.state = "exploring"

# Initialize Robots
robots = [ForagingRobot() for _ in range(NUM_ROBOTS)]

food_counts = []
pheromone_levels = []

plt.ion()
fig, ax = plt.subplots(figsize=(6,6))

# Simulation Loop
for step in range(MAX_STEPS):
    decay = PHEROMONE_DECAY if step < MAX_STEPS/2 else PHEROMONE_DECAY_PERTURBED
    pheromone_grid *= decay

    carrying = 0
    for bot in robots:
        bot.step()
        if bot.state == "returning":
            carrying += 1

    food_counts.append(carrying)
    pheromone_levels.append(np.mean(pheromone_grid))

    # Live visualization every 10 steps
    if step % 10 == 0:
        ax.clear()
        ax.imshow(pheromone_grid, cmap='inferno', origin='lower')
        ax.scatter(nest_position[1], nest_position[0], color='green', s=100, label='Nest')
        for f in food_positions:
            ax.scatter(f[1], f[0], color='red', s=100)
        for bot in robots:
            color = 'cyan' if bot.state == "exploring" else 'blue'
            ax.scatter(bot.position[1], bot.position[0], color=color, s=30)
        ax.set_title(f"Swarm Foraging (Step {step})")
        ax.legend(["Nest", "Food"], loc='upper right')
        plt.pause(0.01)

plt.ioff()
plt.show()


plt.figure(figsize=(10,5))
plt.plot(food_counts, label="Robots carrying food")
plt.plot(pheromone_levels, label="Avg pheromone level")
plt.axvline(MAX_STEPS/2, color='r', linestyle='--', label='Perturbation starts')
plt.xlabel("Time step")
plt.ylabel("Collective metrics")
plt.legend()
plt.title("Multi-Food Swarm Trail Formation (with Perturbation)")
plt.show()
