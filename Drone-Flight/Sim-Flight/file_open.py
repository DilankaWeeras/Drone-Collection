mission_pts = []
def read_add_waypoints1():
    #clear_drone_cmds()

    print("Opening Locations")
    file_loc = open('locations.txt', 'r')
    
    line = file_loc.readline()
    line_split = line.split(',')
    full_altitude = int(line_split[0])
    full_yaw = int(line_split[1])
    print("Altitude = " + str(full_altitude))
    print("Yaw = " + str(full_yaw))
    line = file_loc.readline()
    line_split = line.split(',')
    rows = int(line_split[0])
    cols = (line_split[1])
    print("Rows = " + str(rows))
    print("Colums = " + str(cols))

    for line in range(2,len()):
        mission_pts.append(line)
    
    file_loc.close()
    print(mission_pts)

def read_add_waypoints():
    #clear_drone_cmds()

    print("Opening Locations")
    file_loc = open('locations.txt', 'r')
    
    lines = file_loc.readlines()
    line_split = lines[0].split(',')
    full_altitude = int(line_split[0])
    full_yaw = int(line_split[1])
    print("Altitude = " + str(full_altitude))
    print("Yaw = " + str(full_yaw))
    line = file_loc.readline()
    line_split = lines[1].split(',')
    rows = int(line_split[0])
    cols = (line_split[1])
    print("Rows = " + str(rows))
    print("Colums = " + str(cols))

    for line in lines[2:]:
        line_split = line.split(',')
        print("Adding waypoint at: " + line)
        line_split[0] = float(line_split[0])
        line_split[1] = float(line_split[1])
        mission_pts.append(line_split)
    
    file_loc.close()
    print(mission_pts)

read_add_waypoints()