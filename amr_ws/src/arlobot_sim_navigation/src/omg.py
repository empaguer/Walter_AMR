import rospkg
rospack = rospkg.RosPack()
workspace = 'amr_ws'
package = 'arlobot_sim_navigation'
package_path = rospack.get_path(package)
print(package_path)
map = 'worldd'
new = package_path + '/coordinates/' + map + '.csv'
print(new)