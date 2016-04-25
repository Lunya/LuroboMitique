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
def smooth_point_move(point, ref_point, value=0.1):
	if point.x < ref_point.x:
		point.x += value
	elif point.x > ref_point.x:
		point.x -= value
	if point.y < ref_point.y:
		point.y += value
	elif point.y > ref_point.y:
		point.y -= value
	if point.z < ref_point.z:
		point.z += value
	elif point.z > ref_point.z:
		point.z -= value


def loop_function(events, **args):
	for event in events:
		"""if event.type == pygame.MOUSEBUTTONUP:
			if event.button == 1:
				position = interface.circle_displace((200, 200), 80)"""
	keys = args['key_pressed']
	if keys[pygame.K_c]:
		interface.circle_displace((100, 100), 50)
	if keys[pygame.K_r]:
		robot.base_pos()
	if keys[pygame.K_b]:
		#robot.height += 0.6
		ref_center_point.z += 0.6
		#robot.base_pos()
	if keys[pygame.K_n]:
		#robot.height -= 0.6
		ref_center_point.z -= 0.6
		#robot.base_pos()
	if keys[pygame.K_UP]:
		robot.holonom_walk(0.1, 0)
	if keys[pygame.K_DOWN]:
		robot.holonom_walk(-0.1, 0)
	if keys[pygame.K_LEFT]:
		robot.rotation(-0.1, 5)
	if keys[pygame.K_RIGHT]:
		robot.rotation(0.1, 5)


	position = interface.circle_displace((200, 200), 80)
	#robot.center_point = Point(position+(0,))*50.0
	ref_center_point = Point(position+(0,))*20.0
	interface.print_text("Pos: {}".format(robot.center_point), (300, 300))

	
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

	
	treshold = 0.16
	move_X = joystick.get_axis(3)
	move_Y = joystick.get_axis(4)

	if move_X != 0 and move_Y != 0:
		rotate_angle = cartesian_to_polar(Point(joystick.get_axis(4), -joystick.get_axis(3), 0.0))
		rotate_angle = math.degrees(rotate_angle - math.pi)
	else:
		rotate_angle = 0
	interface.print_text("angle {}".format(rotate_angle), (20, 100))
	interface.print_text("force {}".format(math.sqrt((move_X**2) + (move_Y**2))), (20, 120))

	if move_X < treshold and move_X > -treshold:
		move_X = 0
	if move_Y < treshold and move_Y > -treshold:
		move_Y = 0


	interface.print_text("Move X {}".format(move_X), (400,200))
	interface.print_text("Move Y {}".format(move_Y), (450,250))
	force = math.sqrt(move_X**2 + move_Y**2)
	if force != 0:
		robot.holonom_walk(force * 0.1, rotate_angle)

	rotate_X = joystick.get_axis(0)
	rotate_Y = joystick.get_axis(1)
	if rotate_X < treshold and rotate_X > -treshold:
		rotate_X = 0
	if rotate_Y < treshold and rotate_Y > -treshold:
		rotate_Y = 0

	if (rotate_X != 0):
		robot.rotation(rotate_X*0.1, 5)
	robot.center_point.z += rotate_Y * 0.4
	interface.print_text("robot angle {}".format(robot._rotate_angle), (400,220))
	
	#robot.center_point.x = joystick.get_axis(0) * 30
	#robot.center_point.y = joystick.get_axis(1) * 30
	
	#smooth_point_move(robot.center_point, ref_center_point, 1.0)
	
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
