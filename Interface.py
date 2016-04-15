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

		self.font = pygame.font.SysFont('arial', 24)

		self.text_infos = self.font.render('C : move center mode\nH: holonome displacment', True, self.menu_color)

	def stop(self):
		'''
			Stops the program launched by run function
			and quit pygame
		'''
		self.running = False
		pygame.joystick.quit()
		pygame.quit()

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
			# other events request
			if 'key_pressed' in args:
				args['key_pressed'] = pygame.key.get_pressed()
			if 'joystick' in args:
				args['joystick'] = self.joystick


			self.screen.fill(self.background_color)
			function(events, **args)
			pygame.display.flip()

	def circle_displace(self, center, radius):
		'''
			Draw a circle used as a command, 
			returns the position between -1 and 1 of
			the mouse inside on X and Y axis

			position : tuple (X, Y)
		'''
		pygame.draw.circle(self.screen, self.menu_color, center, radius)
		#pygame.draw.rect(self.screen, self.menu_color, position + (diameter, diameter))

	def show_info(self, position):
		self.screen.blit(self.text_infos, position)

	def print_text(self, text, position):
		self.screen.blit(self.font.render(text, True, self.menu_color), position)