# Element
class Packet:
    def accept(self, visitor):
        visitor.visit(self)

# Concrete element
class TCPPacket(Packet):
    def __init__(self, source_port, destination_port, data):
        self.source_port = source_port
        self.destination_port = destination_port
        self.data = data

# Concrete element
class UDPPacket(Packet):
    def __init__(self, source_port, destination_port, data):
        self.source_port = source_port
        self.destination_port = destination_port
        self.data = data

# Visitor
class PacketVisitor:
    def visit(self, packet):
        pass

# Concrete Visitor
class TCPVisitor(PacketVisitor):
    def visit(self, packet):
        if isinstance(packet, TCPPacket):
            print("Processing TCP packet with source port", packet.source_port, "and destination port", packet.destination_port)
            # process the data in the packet
        else:
            print("TCPVisitor cannot process packet of type", type(packet).__name__)

# Concrete Visitor
class UDPVisitor(PacketVisitor):
    def visit(self, packet):
        if isinstance(packet, UDPPacket):
            print("Processing UDP packet with source port", packet.source_port, "and destination port", packet.destination_port)
            # process the data in the packet
        else:
            print("UDPVisitor cannot process packet of type", type(packet).__name__)

# Concrete elements
packets = [TCPPacket(80, 443, "HTTP data"), UDPPacket(53, 53, "DNS data")]

# Concrete visitors
visitors = [TCPVisitor(), UDPVisitor()]

for packet in packets:
    for visitor in visitors:
        packet.accept(visitor)