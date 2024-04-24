import rclpy
from modifiedpublisherfunction import ModifiedMinimalPublisher 
from modifiedsubscriberfunction import ModifiedMinimalSubscriber

def main_loop(args=None):
    minimal_subscriber = ModifiedMinimalSubscriber()
    logical_loop_publisher = ModifiedMinimalPublisher()

    # Check if detect_collision variable is True or False

    detect_collision = True  

    if detect_collision:
        logical_loop_publisher.publish_permission(True)
    else:
        logical_loop_publisher.publish_permission(False)

    minimal_subscriber.destroy_node()
    logical_loop_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
