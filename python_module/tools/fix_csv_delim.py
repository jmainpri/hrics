with open("/home/rafi/Desktop/classes/traj_class_1.csv", 'r') as f:
    with open("temp.csv", 'w') as t:
        for line in f:
            new_line = line.replace(";",",")
            t.write(new_line)
