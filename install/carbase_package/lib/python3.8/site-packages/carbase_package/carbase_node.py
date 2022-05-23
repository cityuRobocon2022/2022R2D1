from .modules.carbase_controller.carbase_controller import CarbaseController

import rclpy

def main(args = None):
    rclpy.init(args = args)

    carbase_controller = CarbaseController()

    rclpy.spin(carbase_controller)

    carbase_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()