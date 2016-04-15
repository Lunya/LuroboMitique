from Point import *
from Interface import *

interface = Interface(800, 600)

def loop_function(events, **args):
	for event in events:
		if event.type == pygame.MOUSEBUTTONUP:
			if event.button == 1:
				interface.stop()


if __name__ == '__main__':
	'''p1 = Point(1, 2, 3)
	p2 = Point([4, 5, 6])
	p3 = Point({"x":7, "y":8, "z":9})
	p4 = Point((10.001, 111.000, -12))'''

	interface.init()
	interface.run(loop_function)
