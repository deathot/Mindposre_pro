import os

import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
import cv2
import acl
from PIL import Image, ImageDraw, ImageFont

from .acllite.acllite_imageproc import AclLiteImageProc
from .acllite.acllite_model import AclLiteModel
from .acllite.acllite_image import AclLiteImage
from .acllite.acllite_resource import resource_list
from .acllite import acllite_utils as utils


# AclLiteResource类，包含初始化acl、释放acl资源功能
class AclLiteResource:
    """
    AclLiteResource
    """

    def __init__(self, device_id=0):
        self.device_id = device_id
        self.context = None
        self.stream = None
        self.run_mode = None

    def init(self):
        """
        init resource
        """
        print("init resource stage:")
        # acl初始化
        ret = acl.init()

        # 指定运算的device
        ret = acl.rt.set_device(self.device_id)
        utils.check_ret("acl.rt.set_device", ret)

        # 创建context
        self.context, ret = acl.rt.create_context(self.device_id)
        utils.check_ret("acl.rt.create_context", ret)

        # 创建stream
        self.stream, ret = acl.rt.create_stream()
        utils.check_ret("acl.rt.create_stream", ret)

        # 获取运行模式
        self.run_mode, ret = acl.rt.get_run_mode()
        utils.check_ret("acl.rt.get_run_mode", ret)

        print("Init resource success")

    def __del__(self):
        print("acl resource release all resource")
        resource_list.destroy()
        if self.stream:
            print("acl resource release stream")
            # 销毁stream
            acl.rt.destroy_stream(self.stream)

        if self.context:
            print("acl resource release context")
            # 释放context
            acl.rt.destroy_context(self.context)

        print("Reset acl device ", self.device_id)
        acl.rt.reset_device(self.device_id)
        # 释放device
        print("Release acl resource success")


class ImageSaverClientNode(Node):
    def __init__(self):
        super().__init__("hdmi_client_node")
        self.client = self.create_client(SetBool, "hdmi_show")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")
        self.request = SetBool.Request()
        self.model_width = 224
        self.model_height = 224
        self.image_net_classes = ["Seashel", "Lighter", "Old Mirror", "Broom", "Ceramic Bowl", "Toothbrush", "Disposable Chopsticks", "Dirty Cloth",
                                  "Newspaper", "Glassware", "Basketball", "Plastic Bottle", "Cardboard", "Glass Bottle", "Metalware", "Hats", "Cans", "Paper",
                                  "Vegetable Leaf", "Orange Peel", "Eggshell", "Banana Peel",
                                  "Battery", "Tablet capsules", "Fluorescent lamp", "Paint bucket"]  # 分类类别

        # Retrieve ROS params
        params = ["image_path", "model_path", "output_folder",
                  "hdmi_sample_folder", "font_path"]
        self.get_params(params)

        # init acl
        self.acl_resource = AclLiteResource()
        self.acl_resource.init()

    def get_params(self, params):
        """
        声明ROS Params
        Args:
            params: list of strings, parameters in config file
        Return:
            None
        """
        for param_name in params:
            self.declare_parameter(param_name, "")
            param_value = self.get_parameter(param_name).value
            setattr(self, param_name, param_value)
            self.get_logger().info("Using {}: {}".format(param_name, param_value))

    def get_image_net_class(self, class_id):
        """
        根据class_id找出对应的类别，并返回
        Args:
            class_id: numpy.int64
        Return:
            class: string
        """
        self.get_logger().debug("class_id has type: {}".format(type(class_id)))
        self.get_logger().debug("class_id: {}".format(class_id))
        if class_id >= len(self.image_net_classes):
            self.get_logger().warning("In get_image_net_class, detected unknown class_id")
            return "unknown"
        else:
            return self.image_net_classes[class_id]

    def post_process(self, infer_output, image_file):
        """
        后处理，包括：
            1. 获取推理结果的ID
            2. 将推理结果绘制在原图中
            3. 保存推理后的图片
        Args:
            infer_output: list, length of 1, element is a numpy array of shape (1, len(image_net_classes))
            image_file: string, image full path
        Return:
            None
        """
        self.get_logger().info("Enter post_process step..")
        data = infer_output[0]
        vals = data.flatten()
        self.get_logger().debug("In post_process, vals has type of {}".format(type(vals)))
        self.get_logger().debug("In post_process, vals has shape of {}".format(vals.shape))
        self.get_logger().debug("In post_process, vals is {}".format(vals))
        # 获取推理出的id
        top_k = vals.argsort()[-1]
        # 获取推理出id对应的类别
        object_class = self.get_image_net_class(top_k)
        # 拼接输出路径
        output_path = os.path.join(
            self.output_folder, os.path.basename(image_file))
        # 获取原始图片
        origin_image = Image.open(image_file)
        # 绘图
        draw = ImageDraw.Draw(origin_image)
        # 设置字体格式
        font = ImageFont.truetype(self.font_path, size=50)
        text_position = (10, 50)
        # 图片上写入类别
        draw.text(text_position, object_class, font=font, fill=255)
        # 保存推理出的图片到输出路径
        origin_image.save(output_path)

    def save_image(self, image_path):
        # 从image_path获取图片
        dvpp = AclLiteImageProc(self.acl_resource)
        self.get_logger().info("current image_path is: {}".format(image_path))
        image = AclLiteImage(image_path)

        # 图片DVPP解码
        image_input = image.copy_to_dvpp()
        self.get_logger().debug("image_input has type: {}".format(type(image_input)))
        yuv_image = dvpp.jpegd(image_input)

        # 模型推理
        model = AclLiteModel(self.model_path)
        resized_image = dvpp.resize(yuv_image,
                                    self.model_width, self.model_height)
        result = model.execute([resized_image, ])
        self.post_process(result, image_path)

        # 读取推理结果图片
        res_image = AclLiteImage(os.path.join(
            self.output_folder, os.path.basename(image_path)))
        res_image_input = res_image.copy_to_dvpp()
        res_yuv_image = dvpp.jpegd(res_image_input)
        res_yuv_image = res_yuv_image.byte_data_to_np_array()

        # 保存图片
        yuv_image_path = os.path.join(
            self.hdmi_sample_folder, "ut_1920x1080_nv12.yuv")
        with open(yuv_image_path, "wb") as f:
            f.write(res_yuv_image.tobytes())

        # 发送请求信号
        self.request.data = True
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info("Response: {}".format(response.message))
        if response.success:
            self.get_logger().info("Image saved successfully")
        else:
            self.get_logger().info("Failed to save image")
        self.get_logger().info("\n")


def main(args=None):
    rclpy.init(args=args)
    node = ImageSaverClientNode()

    # Iterate through files in the specified folder (image_paths)
    for _, image_file in enumerate(os.listdir(node.image_path)):
        if image_file.endswith(".jpg"):
            image_path = os.path.join(node.image_path, image_file)
            orig_img = cv2.imread(image_path)
            resized_img = cv2.resize(orig_img, (1920, 1080))
            cv2.imwrite(image_path, resized_img)
            node.save_image(image_path)
        else:
            node.get_logger().warning(
                "Detect one file that is not of jpg format: {}, skipped".format(image_file))

    rclpy.shutdown()


if __name__ == "__main__":
    main()
