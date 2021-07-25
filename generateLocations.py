from random import randint

pathMARL = "./MARL/GameMaps/libre/"
pathBaseline = "./Baseline/GameMaps/libre/"
name = "libre"
map_file = open(pathMARL + name + ".map2", 'r')


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
n = 200
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
            index = index + 1
            print(index)


fileMARL = open(pathMARL + name + "-" + str(n) + ".loc2", "w")
fileBaseline = open(pathBaseline + name + "-" + str(n) + ".loc2", "w")
for agent in agents:
    fileMARL.write(str(agent) + "\n")
    fileBaseline.write(str(agent) + "\n")
fileMARL.close()
fileBaseline.close()
