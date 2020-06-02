import os

if __name__ == '__main__':
    file_path = "D:/Mr.Niro/School/graduation project/Data/2011_09_26_drive_0020_sync/oxts/"

    velocity = []
    angle = []
    all_file = os.listdir(file_path + "data/")
    for each in all_file:
        file = open(file_path + "data/" + each)
        f = list(file)[0].strip().split()
        velocity.append([float(f[7]), float(f[6]), float(f[10])])
        angle.append([float(f[3]), float(f[4]), float(f[5])])
        file.close()

    f = open(file_path + "timestamps.txt")
    time = []
    for each in f.readlines():
        time.append(float(each.strip().split(":")[-1]))
    f.close()

    pos = [[0, 0, 0, angle[0][0], angle[0][1], angle[0][2]]]
    for i in range(1, len(time)):
        if time[i] < time[i-1]:
            delta_t = 60 - time[i-1] + time[i]
        else:
            delta_t = time[i] - time[i-1]

        px = (velocity[i][0] + velocity[i-1][0]) / 2 * delta_t
        py = (velocity[i][1] + velocity[i-1][1]) / 2 * delta_t
        pz = (velocity[i][2] + velocity[i-1][2]) / 2 * delta_t

        if i > 1:
            px += float(pos[i - 1][0])
            py += float(pos[i - 1][1])
            pz += float(pos[i - 1][2])

        rx = angle[i][0]
        ry = angle[i][1]
        rz = angle[i][2]

        pos.append([px, py, pz, rx, ry, rz])

    with open(file_path + "move_step.txt", "w") as f:
        for line in pos:
            for each in line:
                f.write(format(each, ".6f") + ' ')
            f.write("\n")
    f.close()
