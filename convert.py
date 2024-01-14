cam_coordinates = [(26,11), (24,632), (237,10), (238, 631)] #do not forget x and y are replaced
robot_coordinates = [(0.66, -0.37), (0.67, 0.35), (0.95, -0.40), (0.67, 0.25)]

x_c2 = (cam_coordinates[0][0] + cam_coordinates[1][0]) / 2
x_c1 = (cam_coordinates[2][0] + cam_coordinates[3][0]) / 2
x_r2 = (robot_coordinates[0][0] + robot_coordinates[1][0]) / 2
x_r1 = (robot_coordinates[2][0] + robot_coordinates[3][0]) / 2

y_c2 = (cam_coordinates[0][1] + cam_coordinates[2][1]) / 2
y_c1 = (cam_coordinates[1][1] + cam_coordinates[3][1]) / 2
y_r2 = (robot_coordinates[0][1] + robot_coordinates[2][1]) / 2
y_r1 = (robot_coordinates[1][1] + robot_coordinates[3][1]) / 2

def convert(x, y):
    x_replaced = y
    y_replaced = x

    x_new = x_r1 - (((x_r1 - x_r2) * (x_c1 - x_replaced)) / (x_c1 - x_c2))
    y_new = y_r1 - (((y_r1 - y_r2) * (y_c1 - y_replaced)) / (y_c1 - y_c2))

    return (x_new, y_new)

object_coordinates = (438,198) 
new_robot_coordinates = convert(object_coordinates[0], object_coordinates[1])

print(new_robot_coordinates)
