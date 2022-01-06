import Model
import Path
from generate_trajectories import init_graphs, init_model, return_drone_from_flight_number
from main import generate_scenarios
import BlueskySCNTools

graph_file_path = "graph_files/total_graph.graphml"
# graph_file_path = "graph_files/geo_data/crs_epsg_32633/road_network/crs_4326_cleaned_simplified_network/cleaned_simplified.graphml"
# drone_list_file_path = 'graph_files/drones.txt'
drone_list_file_path = 'graph_files/very_low_intention.csv'
# output_file = "PLNE OUTPUT/10 drones 5 traj/FN_traj.dat"
# output_scenario = "PLNE OUTPUT/10 drones 5 traj/scenario.scn"
output_file = "PLNE OUTPUT/FN_traj.dat"
output_scenario = "PLNE OUTPUT/scenario.scn"
protection_area = 30
# vertical_protection_area = 7.62 # 25 ft
nb_FL = 10
FL_sep = 7.62
FL_step = 25
max_delta_fl = nb_FL-1
delay_max = 60
delay_step = 10

# Check for conflicts
print("Initialising")
graph, graph_dual = init_graphs(graph_file_path)
model = init_model(graph, graph_dual, drone_list_file_path, protection_area)
chosen_traj = [0, 0, 0, 0, 0, 1, 0, 0, 0, 1]
# selected_traj = [1, 10, 11, 16, 21, 30, 35, 40, 41, 46]
# selected_traj = [1 6 11 16 21 26 31 36 41 46]
selected_traj = [1, 6, 11, 16, 21, 26, 31, 36, 41, 46]
delay = [0, 0, 3, 3, 0, 0, 6, 1, 5, 0]
fl = [1, 1, 1, 1, 1, 1, 2, 2, 1, 1]
alts = dict()


with open(output_file, 'r') as file:
    lines = file.readlines()
    for i in range(len(selected_traj)):
        # find the corresponding drone
        current_drone_FN = None
        path_list = None
        for j in range(len(lines)):
            if j % 2 == 1:
                continue
            traj_id = lines[j].split()[1]
            # print(traj_id, selected_traj[i])
            if str(selected_traj[i]-1) == traj_id:
                current_drone_FN = lines[j].split()[0]
                path_list = lines[j+1]
                break
            # print(current_drone_FN)
        for index, drone in enumerate(model.droneList):
            if drone.flight_number == current_drone_FN:
                path_list = path_list.split()
                # set it's dep time with delay
                print(drone.flight_number, delay[i], drone.dep_time)
                drone.dep_time = drone.dep_time + delay[i] * delay_step
                path_object = Path.Path(drone.dep_time, [])
                path_object.set_path(path_list, graph, graph_dual, drone)
                drone.path_object = path_object
                alts[drone.flight_number] = []
                for _k in range(len(path_list)):
                    alts[drone.flight_number].append(FL_step * fl[index])
                alts[drone.flight_number][0] = FL_step
                alts[drone.flight_number][-1] = FL_step
                break



    print("Generating Bluesky SCN")
    scenario_dict = generate_scenarios(model, alts=alts)
    bst = BlueskySCNTools.BlueskySCNTools()
    bst.Dict2Scn(output_scenario, scenario_dict)
    # Add a line to enable ASAS in Bluesky
    with open(output_scenario, 'r+') as f:
        content = f.read()
        f.seek(0, 0)
        f.write("00:00:00>CDMETHOD STATEBASED")
        f.write("\n00:00:00>DTLOOK 20")
        f.write("\n")
        f.write(content)

        # set it's path

# for traj in chosen_traj:
#     drone = return_drone_from_flight_number()
#     pt = Path.Path(drone.dep_time, [])
#     pt.set_path(traj)


# Generate SCN
