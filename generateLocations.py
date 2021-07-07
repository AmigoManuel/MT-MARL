from random import randint

map_file = open("C:/Users/diazu/MT-MARL/MARL/GameMaps/test88.map2", 'r')


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


lines = map_file.read().splitlines()
width = len(lines[0]) - 1
height = len(lines) - 1
n = 400
index = 0

mapped_loc = []
agents = []

while index < n:
    start_loc = (randint(0, width), randint(0, height))

    if lines[start_loc[1]][start_loc[0]] == "." and start_loc not in mapped_loc:
        goal_loc = (randint(0, width), randint(0, height))

        if lines[goal_loc[1]][goal_loc[0]] == "." and start_loc != goal_loc and goal_loc not in mapped_loc:
            mapped_loc.append(start_loc)
            mapped_loc.append(goal_loc)
            agents.append(Agent(index, start_loc[0], start_loc[1], goal_loc[0], goal_loc[1]))
            print(len(agents))
            index = index + 1

for agent in agents:
    print(agent)
