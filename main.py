from Point import *
from Interface import *
from Robot import *

interface = Interface(800, 600)
robot = Robot('robotConfig3.json')
i = 0.005
def loop_function(events, **args):
	for event in events:
		if event.type == pygame.MOUSEBUTTONUP:
			if event.button == 1:
				interface.stop()
	keys = args['key_pressed']
	if keys[pygame.K_c]:
		interface.circle_displace((100, 100), 50)
	elif keys[pygame.K_b]:
		robot.height += 0.6
		robot._calculate_base_pos()
		#robot.base_pos()
	elif keys[pygame.K_n]:
		robot.height -= 0.6
		robot._calculate_base_pos()
		#robot.base_pos()

	joystick = args['joystick']
	for i in range(joystick.get_numbuttons()):
		if joystick.get_button(i) == True:
			print i

	if joystick.get_button(6):
		interface.show_info((50, 20))
		robot.height -= 0.6
		robot._calculate_base_pos()
		#robot.base_pos()
	elif joystick.get_button(7):
		interface.show_info((50, 20))
		robot.height += 0.6
		robot._calculate_base_pos()
		#robot.base_pos()

	robot.holonom_walk(i, joystick.get_axis(2), 0)

	robot.center_point.x = joystick.get_axis(0) * 30
	robot.center_point.y = joystick.get_axis(1) * 30
	robot.move()

	"""for i in range(joystick.get_numbuttons()):
		if joystick.get_button(i) == True:
			print i"""


	interface.show_info((50, 20))


if __name__ == '__main__':
	'''p1 = Point(1, 2, 3)
	p2 = Point([4, 5, 6])
	p3 = Point({"x":7, "y":8, "z":9})
	p4 = Point((10.001, 111.000, -12))'''

	interface.init()
	interface.run(loop_function, key_pressed=[], joystick=None)
