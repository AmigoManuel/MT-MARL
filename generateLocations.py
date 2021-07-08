from random import randint

path = "C:/Users/diazu/MT-MARL/MARL/GameMaps/"
map_name = "test10"
map_file = open(path + map_name + ".map2", 'r')

n = 10


class Agent:
    def __init__(self, a, startx, starty, goalx, goaly):
        self.a = a
        self.startx = startx
        self.starty = starty
        self.goalx = goalx
        self.goaly = goaly

    def __str__(self):
        return "a " + str(self.a) + "\n" \
               + "goaly " + str(self.goaly) + "\n" \
               + "goalx " + str(self.goalx) + "\n" \
               + "starty " + str(self.starty) + "\n" \
               + "startx " + str(self.startx)


def strip_lines():
    lines = map_file.readlines()
    for i in range(len(lines)):
        lines[i] = lines[i].strip()
    return lines


def scan_availables(lines):
    availables = []
    for y in range(len(lines)):
        line = lines[y]
        for x in range(len(line)):
            char = line[x]
            if char == ".":
                availables.append((y, x))
    return availables


def generate_locations(availables):
    agents = []
    for i in range(10):
        start_index = randint(0, len(availables))
        start = availables[start_index]
        availables.remove(start)
        goal_index = randint(0, len(availables))
        goal = availables[goal_index]
        availables.remove(goal)
        agents.append(
            Agent(
                i,
                goal[0],
                goal[1],
                start[0],
                start[1]
            )
        )
    return agents


def write_to_file(agents):
    file = open(path + map_name + ".loc2", "w")
    for agent in agents:
        file.write(str(agent) + "\n")
    file.close()


if __name__ == '__main__':
    lines = strip_lines()
    width = len(lines[0]) - 1
    height = len(lines) - 1
    availables = scan_availables(lines)
    agents = generate_locations(availables)
    write_to_file(agents)
