import os
import sys

import numpy as np
import rclpy
import pytest
from example_interfaces.srv import SetBool

from service_client.client import ImageSaverClientNode
from service_client.server import ImageSaverNode

rclpy.init(args=sys.argv)


@pytest.fixture
def client_node():
    return ImageSaverClientNode()


@pytest.fixture
def server_node():
    return ImageSaverNode()


def test_get_image_net_class(client_node):
    assert client_node.get_image_net_class(0) == "Seashel"
    assert client_node.get_image_net_class(1) == "Lighter"
    assert client_node.get_image_net_class(2) == "Old Mirror"
    assert client_node.get_image_net_class(3) == "Broom"
    assert client_node.get_image_net_class(4) == "Ceramic Bowl"
    assert client_node.get_image_net_class(5) == "Toothbrush"
    assert client_node.get_image_net_class(6) == "Disposable Chopsticks"
    assert client_node.get_image_net_class(7) == "Dirty Cloth"
    assert client_node.get_image_net_class(8) == "Newspaper"
    assert client_node.get_image_net_class(9) == "Glassware"
    assert client_node.get_image_net_class(10) == "Basketball"
    assert client_node.get_image_net_class(11) == "Plastic Bottle"
    assert client_node.get_image_net_class(12) == "Cardboard"
    assert client_node.get_image_net_class(13) == "Glass Bottle"
    assert client_node.get_image_net_class(14) == "Metalware"
    assert client_node.get_image_net_class(15) == "Hats"
    assert client_node.get_image_net_class(16) == "Cans"
    assert client_node.get_image_net_class(17) == "Paper"
    assert client_node.get_image_net_class(18) == "Vegetable Leaf"
    assert client_node.get_image_net_class(19) == "Orange Peel"
    assert client_node.get_image_net_class(20) == "Eggshell"
    assert client_node.get_image_net_class(21) == "Banana Peel"
    assert client_node.get_image_net_class(22) == "Battery"
    assert client_node.get_image_net_class(23) == "Tablet capsules"
    assert client_node.get_image_net_class(24) == "Fluorescent lamp"
    assert client_node.get_image_net_class(25) == "Paint bucket"
    assert client_node.get_image_net_class(26) == "unknown"


def test_post_process(client_node):
    infer_result = np.zeros(27)
    infer_result[5] += 1
    infer_result = infer_result.reshape(1, -1)
    infer_output = [infer_result]
    image_file = os.path.join("data", "test.jpg")
    client_node.post_process(infer_output, image_file)
    assert os.path.exists(os.path.join(client_node.output_folder, "test.jpg"))


def test_save_image(client_node):
    image_path = os.path.join("data", "test.jpg")
    client_node.save_image(image_path)
    assert os.path.exists(os.path.join(
        client_node.hdmi_sample_folder, "ut_1920x1080_nv12.yuv"))


def test_save_image_callback(server_node):
    request = SetBool.Request()
    response = SetBool.Response()
    assert server_node.save_image_callback(request, response) == response
    assert response.success == True
    assert response.message == 'Image saved successfully'
