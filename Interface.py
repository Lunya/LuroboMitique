from Point import *
from Robot import rotateXY

import pygame


class Interface(object):
	def __init__(self, width, height):
		self.width = width
		self.height = height
		self.running = False
		self.clock = pygame.time.Clock()
		self.actions = {}
		self.fps = 120
		self.n = 0
		# class variables
		self.background_color = (220, 220, 220)
		self.menu_color = (80, 80, 80)
		self.interest_color = (255, 80, 0)

		# circles displace array
		self._circles_displace = []



	def init(self):
		'''
			This function will be used to initiate all
			program utilities, It will be called before run
		'''
		pygame.init()
		pygame.joystick.init()
		self.screen = pygame.display.set_mode((self.width, self.height))
		pygame.display.set_caption('LuroboMitique')

		# joystick binding
		if pygame.joystick.get_count() > 0:
			self.joystick = pygame.joystick.Joystick(0)
			self.joystick.init()
		else:
			self.joystick = None

		# text initialising
		self.font = pygame.font.SysFont('arial', 24)

	def stop(self):
		'''
			Stops the program launched by run function
			and quit pygame
		'''
		self.running = False
		pygame.joystick.quit()

	def run(self, function, **args):
		'''
			Run program and run function in infinite loop
			function will be called with two arguments:
				an array of events
				keywords arguments passed at run function
		'''
		self.running = True
		while self.running:
			self.clock.tick(self.fps)
			events = pygame.event.get()
			for event in events:
				if event.type == pygame.QUIT:
					self.running = False
				elif event.type == pygame.KEYDOWN:
					if event.key == pygame.K_q:
						self.running = False

			# clean the screen
			self.screen.fill(self.background_color)
			#mouse position
			self.mouse_pos = pygame.mouse.get_pos()

			# other events request
			if 'key_pressed' in args:
				args['key_pressed'] = pygame.key.get_pressed()
			if 'joystick' in args:
				args['joystick'] = self.joystick


			function(events, **args)

			# show interface
			for circle in self._circles_displace:
				# if cursor is in the circle
				if ((self.mouse_pos[0] - circle[0][0])**2 + (self.mouse_pos[1] - circle[0][1])**2) <= circle[1]**2:
					self._draw_crosshair(self.mouse_pos, 12)
				else:
					self._draw_crosshair(circle[0])
			pygame.display.flip()

			# reset values
			self._circles_displace = []

		pygame.quit()

	def _draw_crosshair(self, position, size=5):
		'''
			draw a crosshair at the position pos
			of the size size*2
		'''
		pygame.draw.line(
			self.screen, self.interest_color,
			(position[0] + size + 1, position[1]),
			(position[0] - size, position[1])
		)
		pygame.draw.line(
			self.screen, self.interest_color,
			(position[0], position[1] + size + 1),
			(position[0], position[1] - size)
		)


	def circle_displace(self, center, radius):
		'''
			Draw a circle used as a command, 
			returns the position between -1 and 1 of
			the mouse inside on X and Y axis

			position : tuple (X, Y)
			radius : number
		'''
		self._circles_displace.append([center, radius]);
		pygame.draw.circle(self.screen, self.menu_color, center, radius)
		# if cursor is in the circle
		if ((self.mouse_pos[0] - center[0])**2 + (self.mouse_pos[1] - center[1])**2) <= radius**2:
			self._draw_crosshair(self.mouse_pos, 12)
			return (-(float(center[0] - self.mouse_pos[0])) / float(radius), (float(center[1] - self.mouse_pos[1]) / float(radius)))
		else:
			self._draw_crosshair(center)
			return (0, 0)
		#pygame.draw.rect(self.screen, self.menu_color, position + (diameter, diameter))

	def print_text(self, text, position):
		self.screen.blit(self.font.render(text, True, self.menu_color), position)

