import time
import csv
import os


class RunTime:
    def __init__(self):
        self.init_time = time.time()
        self.last_time = self.init_time
        self.map_time = 0
        self.search_time = 0
        self.total_time = 0
        self.route_len = 0
        self.steps_num = 0
        self.states_num = 0 # Number of visited states
        directory = os.path.dirname(__file__)
        self.csv = os.path.join(directory, "output", "plan_times.csv")
        self.headers = ["Map time (s)", "Search time (s)", "Total time (s)", "Route length (cm)", "Number of steps", "States visited"]

    def get_map_time(self):
        curr_time = time.time()
        self.map_time = curr_time - self.last_time
        self.print_time(curr_time, self.map_time, description="Map processing time")
        return

    def get_search_time(self):
        curr_time = time.time()
        self.search_time = curr_time - self.last_time
        self.print_time(curr_time, self.search_time, description="Search time")
        return

    def print_time(self, curr_time, dif_time, description):
        self.last_time = curr_time
        if dif_time <= 0:
            print(description + " less than 1 ms")
        else:
            print(description + " %.3f s" % round(dif_time, 3))
        return

    def get_total_time(self):
        self.total_time = time.time() - self.init_time
        print("Total execution time: %.3f s" % round(self.total_time, 3))
        print("Check %f" % (self.total_time - self.map_time - self.search_time))
        return

    def save_len(self, length, steps, states):
        self.route_len = length
        self.steps_num = steps
        self.states_num = states
        print("Length of the route: %.2f cm" % self.route_len)
        print("Number of steps: %i" % self.steps_num)
        print("Number of visited states: %i" % self.states_num)
        return

    def write_data(self):
        self.get_total_time()
        row = [self.map_time, self.search_time, self.total_time, self.route_len, self.steps_num, self.states_num]        
        if not os.path.exists(self.csv):
            self.clear_csv()
        with open(self.csv, 'a', newline='') as f:
            # create the csv writer
            writer = csv.writer(f)

            # write a row to the csv file
            writer.writerow(row)
        return

    def clear_csv(self):
        with open(self.csv, 'w+', newline='') as f:
            # create the csv writer
            writer = csv.writer(f)

            # write a row to the csv file
            writer.writerow(self.headers)

    def __str__(self):
        return self.last_time
