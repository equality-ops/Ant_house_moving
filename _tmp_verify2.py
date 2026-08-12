class A:
    __slots__ = ('__x', 'y')
    def __init__(self):
        self.__x = 1
        self.y = 2

class B:
    __slots__ = ('_B__x', 'y')
    def __init__(self):
        self.__x = 1
        self.y = 2

a = A()
b = B()
print('A.__slots__ =', A.__slots__)
print('B.__slots__ =', B.__slots__)
print('A has _A__x:', hasattr(a, '_A__x'), '| A has __x:', hasattr(a, '__x'))
print('B has _B__x:', hasattr(b, '_B__x'))
