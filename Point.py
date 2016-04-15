

class Point(object):
	def __init__(self, x=0, y=0, z=0):
		if type(x) is type(self):
			self.x = x.x
			self.y = x.y
			self.z = x.z
		elif type(x) is dict:
			self.x = x["x"]
			self.y = x["y"]
			self.z = x["z"]
		elif type(x) is list:
			self.x = x[0]
			self.y = x[1]
			self.z = x[2]
		elif type(x) is tuple:
			self.x = x[0]
			self.y = x[1]
			self.z = x[2]
		elif type(x) is int or type(x) is float or type(x) is long:
			self.x = x
			self.y = y
			self.z = z
		else:
			raise TypeError("Bad argument type [ {:s} ] expected [int], [float], [tuple], [list] or [dict]")

	def __repr__(self):
		return "Z:{: 3.3f} Y:{: 3.3f} Z:{: 3.3f}".format(self.x, self.y, self.z)

	def copy(self):
		return Point(self.x, self.y, self.z)

	def __neg__(self):
		return Point(-self.x, -self.y, -self.z)

	def __pos__(self):
		return Point(self)

	def __add__(self, obj): # Point + obj
		if type(obj) is type(self):
			return Point(
				self.x + obj.x,
				self.y + obj.y,
				self.z + obj.z
			)
		elif type(obj) is int or type(obj) is float or type(obj) is long:
			return Point(
				self.x + obj,
				self.y + obj,
				self.z + obj
			)
		else:
			raise TypeError("Bad argument, type expected [int], [float] or class [Point]")

	def __radd__(self, obj): # obj + Point
		return self + obj

	def __sub__(self, obj):
		if type(obj) is type(self):
			return Point(
				self.x - obj.x,
				self.y - obj.y,
				self.z - obj.z
			)
		elif type(obj) is int or type(obj) is float or type(obj) is long:
			return Point(
				self.x - obj,
				self.y - obj,
				self.z - obj
			)
		else:
			raise TypeError("Bad argument, type expected [int], [float] or [class Point]")

	def __div__(self, obj):
		if type(obj) is type(self):
			return Point(
				self.x / obj.x,
				self.y / obj.y,
				self.z / obj.z
			)
		elif type(obj) is int or type(obj) is float or type(obj) is long:
			return Point(
				self.x / obj,
				self.y / obj,
				self.z / obj
			)
		else:
			raise TypeError("Bad argument, type expected [int], [float] or [class Point]")

	def __mul__(self, obj):
		if type(obj) is type(self):
			return Point(
				self.x * obj.x,
				self.y * obj.y,
				self.z * obj.z
			)
		elif type(obj) is int or type(obj) is float or type(obj) is long:
			return Point(
				self.x * obj,
				self.y * obj,
				self.z * obj
			)
		else:
			raise TypeError("Bad argument, type expected [int], [float] or [class Point]")

'''
Overloaded operators
__pow__(1) **
__truediv__(1) /
__floordiv__(1) //
__modulo__(1) %

__iadd__(1) +=
__isub__(1) -= ... *= /= //= %=
__ipow__(1, [modulo]) **=

__not__() ~
__neg__() -
__pos__() +
__int__() int(x)
__long__() long(x)
__float__() float(x)

__rpow__(1) same as pow but with class at right, not at left
__rmul__(1) ...
'''