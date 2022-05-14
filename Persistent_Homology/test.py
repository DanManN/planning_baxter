class const:
    def __init__(self, a):
        self.a = a

    def A(self):
        return self.a

class node:
    def __init__(self, const):
        self.const = const
        self.b = const.a

    def B(self):
        return self.const.a

    def update(self):
        self.const.a = 7

    def C(self):
        return self.b



node1 = node(const(1))

print(node1.b)
print(node1.B())
node1.update()
print(node1.b)
print(node1.B())
print(node1.C())
