from Point import *
from Interface import *
from Robot import *

interface = Interface(1200, 800)
robot = Robot('robotConfig3.json')

def cartesian_to_polar(p):
	if p.x > 0 and p.y >=0:
		return math.atan(p.y / p.x)
	elif p.x > 0 and p.y < 0:
		return math.atan(p.y / p.x) + 2.0 * math.pi
	elif p.x < 0:
		return math.atan(p.y / p.x) + math.pi
	elif p.x == 0 and p.y > 0:
		return math.pi / 2.0
	elif p.x == 0 and p.y < 0:
		return (3.0 * math.pi) / 2.0
	else:
		return math.pi / 2.0


ref_center_point = Point(0, 0, -120)


def loop_function(events, **args):
	for event in events:
		"""if event.type == pygame.MOUSEBUTTONUP:
			if event.button == 1:
				position = interface.circle_displace((200, 200), 80)"""
	keys = args['key_pressed']
	if keys[pygame.K_r]:
		robot.base_pos()
	if keys[pygame.K_b]:
		#robot.height += 0.6
		robot.height += 0.6
		#robot.base_pos()
	if keys[pygame.K_n]:
		#robot.height -= 0.6
		robot.height -= 0.6
		#robot.base_pos()
	if keys[pygame.K_UP]:
		robot.holonom_walk(0.1, 0)
	if keys[pygame.K_DOWN]:
		robot.holonom_walk(-0.1, 0)
	if keys[pygame.K_LEFT]:
		robot.rotation(-0.1, 5)
	if keys[pygame.K_RIGHT]:
		robot.rotation(0.1, 5)
	
	interface.print_text("Use arrow keys to move robot in 4 directions", (40,500))
	interface.print_text("Use 'N' and 'B' to move vertically the center of robot", (40,530))
	interface.print_text("Use left joystick to control the rotation and the height of the robot", (40,560))
	interface.print_text("Use right joystick to move robot in 4 directions", (40,590))
	interface.print_text("Use 'Q' to quit application", (40,620))

	interface.print_text("Press A button to change leg walk", (40,340))
	interface.print_text("{}".format('Three duplets of legs' if robot._is_two_legs else 'Two triplets of legs'), (40,360))

	interface.print_text("Move the cursor in the circle to move the robot center", (20,80))
	position = interface.circle_displace((200, 200), 80)
	robot.center_point = Point(position+(0,))*50.0
	#ref_center_point = Point(position+(0,))*20.0
	interface.print_text("Center position: {}".format(robot.center_point), (300, 300))

	
	joystick = args['joystick']
	for i in range(joystick.get_numbuttons()):
		if joystick.get_button(i) == True:
			print i

	if joystick.get_button(6):
		robot.height -= 0.6
		robot._calculate_base_pos()
		#robot.base_pos()
	elif joystick.get_button(7):
		robot.height += 0.6
		robot._calculate_base_pos()
		#robot.base_pos()
	elif joystick.get_button(0):
		robot._is_two_legs = not robot._is_two_legs

	
	treshold = 0.16

	rotate_X = joystick.get_axis(0)
	rotate_Y = joystick.get_axis(1)
	if rotate_X < treshold and rotate_X > -treshold:
		rotate_X = 0
	if rotate_Y < treshold and rotate_Y > -treshold:
		rotate_Y = 0

	if (rotate_X != 0):
		robot.rotation(rotate_X*0.1, 5)
	robot.height += rotate_Y * 0.4
	interface.print_text("robot angle {}".format(robot._rotate_angle), (400,220))



	move_X = joystick.get_axis(3)
	move_Y = joystick.get_axis(4)

	if move_X != 0 and move_Y != 0:
		rotate_angle = cartesian_to_polar(Point(joystick.get_axis(4), -joystick.get_axis(3), 0.0))
		rotate_angle = math.degrees(rotate_angle - math.pi)
	else:
		rotate_angle = 0
	interface.print_text("Walk direction {}".format(rotate_angle), (400, 140))
	interface.print_text("Walk speed {}".format(math.sqrt((move_X**2) + (move_Y**2))), (400, 160))

	if move_X < treshold and move_X > -treshold:
		move_X = 0
	if move_Y < treshold and move_Y > -treshold:
		move_Y = 0


	force = math.sqrt(move_X**2 + move_Y**2)
	if force != 0:
		robot.holonom_walk(force * 0.1, rotate_angle-rotate_X*0.1)

	#robot.center_point = ref_center_point
	
	robot.move()

	"""for i in range(joystick.get_numbuttons()):
		if joystick.get_button(i) == True:
			print i"""


if __name__ == '__main__':
	'''p1 = Point(1, 2, 3)
	p2 = Point([4, 5, 6])
	p3 = Point({"x":7, "y":8, "z":9})
	p4 = Point((10.001, 111.000, -12))'''

	interface.init()
	interface.run(loop_function, key_pressed=[], joystick=None)
