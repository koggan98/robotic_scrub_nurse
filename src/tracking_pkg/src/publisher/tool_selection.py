#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ToolSelector(Node):
    def __init__(self):
        super().__init__('tool_selector')
        self.publisher_ = self.create_publisher(String, '/tool_selection', 10)
        self.get_logger().info("ToolSelector node started. Enter a number to publish tool selection.")

    def send_tool_selection(self, tool_number):
        msg = String()
        msg.data = str(tool_number)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published tool selection: '{msg.data}'")

def print_menu():
    print("\n" + "=" * 40)
    print("🛠️  Select a Tool to Pick Up:")
    print("=" * 40)
    print("  0 - Reset")
    print("  1 - Tweezers (long)")
    print("  2 - Hammer")
    print("  3 - Scissors (long)")
    print("  4 - Scissors (short) (LEFT)")
    print("  4 - Scissors (short) (RIGHT)")
    print("  6 - Retractor (small)")
    print("=" * 40)

def main():
    rclpy.init()
    node = ToolSelector()

    try:
        while rclpy.ok():
            print_menu()
            try:
                tool_input = input("Enter tool number (0–6): ").strip()
                if tool_input not in ['0', '1', '2', '3', '4', '5', "6"]:
                    print("❌ Invalid input. Please enter a number between 0 and 6.")
                    continue
                node.send_tool_selection(tool_input)
            except KeyboardInterrupt:
                print("\nExiting.")
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
