from Point import *
from Interface import *
from Robot import *

interface = Interface(800, 600)
robot = Robot('robotConfig3.json')
i = 0.005
def angle_holonome(a, b):
	if a >= 0.0 and b > 0.0: # quart haut droite
		return math.atan(a/b)
	elif a < 0.0 and b >= 0.0: # quart haut gauche
		return math.atan(b/abs(a)) + math.pi/2.0
	elif a <= 0.0 and b < 0.0: # quart bas gauche
		return math.atan(abs(a)/abs(b)) + math.pi
	elif a > 0.0 and b <= 0.0: # quart bas droite
		return math.atan(abs(b)/a) + math.pi*1.5
	else:
		return 0;

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
	elif keys[pygame.K_UP]:
		robot.holonom_walk(i, 10, 0)
	elif keys[pygame.K_DOWN]:
		robot.holonom_walk(i, -10, 0)
	elif keys[pygame.K_LEFT]:
		robot.rotation(i, 20, 5)
	elif keys[pygame.K_RIGHT]:
		robot.rotation(i, -20, 5)

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

	#robot.holonom_walk(i, joystick.get_axis(2), 0)
	rotate_angle = angle_holonome(joystick.get_axis(2), -joystick.get_axis(3))
	rotate_angle = math.degrees(rotate_angle)
	interface.print_text("angle {}".format(rotate_angle), (20, 100))
	interface.print_text("force {}".format(math.sqrt(joystick.get_axis(2)**2 + joystick.get_axis(3)**2)), (20, 120))
	robot.holonom_walk(i, 0.003*math.sqrt(joystick.get_axis(2)**2 + joystick.get_axis(3)**2), rotate_angle)
	robot.rotation(i, joystick.get_axis(0), 1)

	#robot.center_point.x = joystick.get_axis(0) * 30
	#robot.center_point.y = joystick.get_axis(1) * 30
	
	#robot.move()

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
