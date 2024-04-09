# def generate_points(self, point1, point2, num_waypoints):
import matplotlib.pyplot as plt
point1 = [1,1]

point2 = [20,3]
num_waypoints = 20
wpx =[]
wpy =[]
for n in range(num_waypoints):
    ratio = n/(num_waypoints)
    x = point1[0] + abs(point1[0]- point2[0])*ratio
    y = point1[1] + abs(point1[1]- point2[1])*ratio
    wpx.append((x,y,1))
    # wpy.append(y)

# print(len(wpx), len(wpy))
# print(wpx[0],wpx[-1])
# print(wpy[0], )
# plt.scatter(wpx, wpy)   
# plt.show()
print(wpx)
