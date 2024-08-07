import subprocess
import time
import signal

import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool


class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('hdmi_server_node')
        self.srv = self.create_service(
            SetBool, 'hdmi_show', self.save_image_callback)
        self.declare_parameter("hdmi_device", "")
        self.hdmi_device = self.get_parameter("hdmi_device").value
        self.get_logger().info("Using hdmi_device: {}".format(self.hdmi_device))
        self.declare_parameter("hdmi_sample_folder", "")
        self.hdmi_sample_folder = self.get_parameter(
            "hdmi_sample_folder").value
        self.get_logger().info("Using hdmi_sample_folder: {}".format(self.hdmi_sample_folder))

    def save_image_callback(self, request, response):
        # Send response signal

        # Combine the commands using '&&' to execute them sequentially in the same shell
        # The 'source ~/.bashrc' command is included to ensure that the environment variables from ~/.bashrc are loaded properly
        combined_command = "source ~/.bashrc && cd " + self.hdmi_sample_folder + \
            " && ./hdmi_sample " + self.hdmi_device + " 0"

        # Use 'bash -c' to execute the combined command in a new bash shell
        process = subprocess.Popen(
            ["bash", "-c", combined_command], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        try:
            # Give some time for the subprocess to run
            time.sleep(5)

            # Send a SIGINT signal to the subprocess to end it gracefully (similar to Ctrl + C)
            process.send_signal(signal.SIGINT)

            # Wait for the process to finish
            process.wait()

            # Fetch the output if needed
            stdout, stderr = process.communicate()
            stdout = stdout.decode("utf-8")
            stderr = stderr.decode("utf-8")

            # Print the output and any errors (if any)
            self.get_logger().debug("### Standard Output:")
            self.get_logger().debug(stdout)
            self.get_logger().debug("### END Standard Output")
            self.get_logger().debug("\n")
            self.get_logger().debug("### Standard Error:")
            if not stderr:
                self.get_logger().debug("None")
            else:
                self.get_logger().debug(stderr)
            self.get_logger().debug("### END Standard Error")

            # Check the return code to see if the commands executed successfully
            if process.returncode == 0:
                self.get_logger().info("Commands executed successfully.")
            else:
                self.get_logger().error("Commands failed with return code: {}".format(process.returncode))

        except KeyboardInterrupt:
            # If the user interrupts the script (presses Ctrl + C), handle the KeyboardInterrupt
            self.get_logger().error("Subprocess was interrupted by the user.")
            # Terminate the subprocess
            process.terminate()

        response.success = True
        response.message = 'Image saved successfully'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ImageSaverNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
