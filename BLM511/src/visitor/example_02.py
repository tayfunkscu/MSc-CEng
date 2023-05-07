# Element
class Robot:
    def accept(self, visitor):
        visitor.visit(self)

# Concrete Element
class WeldingRobot(Robot):
    def weld(self):
        print("The welding robot is welding!")

# Concrete Element
class PaintingRobot(Robot):
    def paint(self):
        print("The painting robot is painting!")

# Concrete Element
class AssemblyRobot(Robot):
    def assemble(self):
        print("The assembly robot is assembling!")

# Visitor
class Visitor:
    def visit(self, robot):
        pass

# Concrete Visitor
class Programmer(Visitor):
    def visit(self, robot):
        if isinstance(robot, WeldingRobot):
            print("Programming the welding robot.")
        elif isinstance(robot, PaintingRobot):
            print("Programming the painting robot.")
        elif isinstance(robot, AssemblyRobot):
            print("Programming the assembly robot.")

# Concrete Visitor
class Calibrator(Visitor):
    def visit(self, robot):
        if isinstance(robot, WeldingRobot):
            print("Calibrating the welding robot.")
        elif isinstance(robot, PaintingRobot):
            print("Calibrating the painting robot.")
        elif isinstance(robot, AssemblyRobot):
            print("Calibrating the assembly robot.")

# Concrete Visitor
class Inspector(Visitor):
    def visit(self, robot):
        if isinstance(robot, WeldingRobot):
            print("Inspecting the welding robot.")
        elif isinstance(robot, PaintingRobot):
            print("Inspecting the painting robot.")
        elif isinstance(robot, AssemblyRobot):
            print("Inspecting the assembly robot.")

# Concrete elemenets
robots = [WeldingRobot(), PaintingRobot(), AssemblyRobot()]

# Concrete visitors
programmer = Programmer()
calibrator = Calibrator()
inspector = Inspector()

# Visit each robot with each visitor
for robot in robots:
    print(type(robot).__name__)
    robot.accept(programmer)
    robot.accept(calibrator)
    robot.accept(inspector)