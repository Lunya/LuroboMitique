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

	def init(self):
		'''
			This function will be used to initiate all
			program utilities, It will be called before run
		'''
		pygame.init()
		self.screen = pygame.display.set_mode((self.width, self.height))
		pygame.display.set_caption('LuroboMitique')

	def stop(self):
		'''
			Stops the program launched by run function
			and quit pygame
		'''
		self.running = False
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
			function(events, **args)

