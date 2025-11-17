import rclpy
from rclpy.node import Node

from std_msgs.msg import String

"""Egyszerű példa publisher node.

Magyarázat:
 - A Node lesz a folyamat, amely ROS2 kommunikációt végez.
 - create_publisher: létrehoz egy Publisher objektumot egy adott üzenettípussal és topic névvel.
 - create_timer: periodikusan meghív egy callback-et (itt 1 másodpercenként).
 - timer_callback: összeállítja az üzenetet (String) és kiküldi a "chatter" topicra.
 - Logger hívások: info/warn/error szintek demonstrációja.
"""

class Publisher_Node(Node):
    def __init__(self):
        # A szülő konstruktor meghívása beállítja a node nevét a gráfban.
        super().__init__('publisher_node')

        # Publisher létrehozása:
        #   - Üzenettípus: String
        #   - Topic név: "chatter"
        #   - QoS queue méret: 10 (egyszerű megbízható sor)
        self.publisher = self.create_publisher(String, "chatter", 10)

        # Időzítő létrehozása 1.0 másodperces periódussal.
        # Ez 1 Hz-es ütemezést jelent – másodpercenként egyszer fut a callback.
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # Ez a függvény minden timer periódusban lefut.
        # Összeállítjuk a küldendő szöveget.
        message = "Hello, ROS2!"

        # Logoljuk különböző szinteken – vizsgán elég az info általában.
        self.get_logger().info(message)
        # Figyelmeztetési és hibaszintek csak demonstráció:
        self.get_logger().warn(message)
        self.get_logger().error(message)

        # Üzenet objektum létrehozása és feltöltése.
        msg = String()
        msg.data = message

        # Publish hívás: a tényleges adatküldés a ROS kommunikációs réteg felé.
        self.publisher.publish(msg)


def main(args=None):
    # ROS inicializálás (parancssori argumentumok továbbítása).
    rclpy.init(args=args)

    # Node példány létrehozása.
    publisher_node = Publisher_Node()

    # rclpy.spin: eseményciklus – kezeljük a timer-t, publikálást, stb.
    rclpy.spin(publisher_node)

    # Leállításkor erőforrások felszabadítása.
    rclpy.shutdown()

if __name__ == '__main__':
    main()