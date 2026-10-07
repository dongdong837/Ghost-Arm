"""StepFun transport and live RGB capture; no robot motion in this module."""

import base64

import json


import os




import time

from urllib.error import HTTPError, URLError

from urllib.parse import urlparse

from urllib.request import Request, urlopen


def capture_camera():
    """只在 --vision 时订阅实时相机，不读取旧截图。"""
    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    rclpy.init()
    node = rclpy.create_node('robot_ai_camera')
    frames = []
    subscription = node.create_subscription(
        Image, '/camera/image', frames.append, qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + 8.0
        while not frames and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not frames:
            raise ValueError('8 秒内未收到 /camera/image，请先启动仿真。')
        pixels = CvBridge().imgmsg_to_cv2(frames[-1], desired_encoding='bgr8')
        ok, jpeg = cv2.imencode('.jpg', pixels)
        if not ok:
            raise ValueError('相机图像 JPEG 编码失败。')
        return 'data:image/jpeg;base64,' + base64.b64encode(jpeg).decode('ascii')
    finally:
        node.destroy_node()
        rclpy.shutdown()

def call_model(command, config, image=None):
    """只发送请求与可选相机帧；密钥不保存、不打印。"""
    key = os.environ.get('STEPFUN_API_KEY', '').strip()
    if not key:
        raise ValueError('缺少 STEPFUN_API_KEY。请先在终端设置密钥，勿写入代码。')
    base = os.environ.get('STEPFUN_BASE_URL', config['base_url']).rstrip('/')
    url = urlparse(base)
    if url.scheme != 'https' and not (url.scheme == 'http' and url.hostname in {'localhost', '127.0.0.1', '::1'}):
        raise ValueError('API 地址需要 HTTPS；本机服务允许 http://localhost 或 127.0.0.1。')
    model = os.environ.get('STEPFUN_VISION_MODEL' if image else 'STEPFUN_LLM_MODEL',
                           config['vision_model' if image else 'llm_model'])
    prompt = config['system_prompt']
    if image is not None:
        # 视觉模式只描述当前图片，不混入地标导航规则或“请使用 --vision”的提示。
        prompt = (
            '你是机器人相机的视觉问答助手。本条用户消息已经包含实时相机图片。'
            '根据图片回答用户问题，描述实际可见物体的颜色、形状及画面位置。'
            '看不清或无法确定的部分明确说明，不猜测画面外物体或世界坐标。'
            '这是观察模式，不执行导航；涉及移动时告知需另发导航指令。'
            '只返回 JSON 对象：{"action":"answer","text":"你的中文观察结果"}。'
        )
    content = command if image is None else [
        {'type': 'text', 'text': command},
        {'type': 'image_url', 'image_url': {'url': image}}]
    payload = {'model': model, 'messages': [
        {'role': 'system', 'content': prompt}, {'role': 'user', 'content': content}],
        'temperature': 0.0, 'max_tokens': 600}
    request = Request(base + '/chat/completions',
                      data=json.dumps(payload).encode('utf-8'),
                      headers={'Authorization': 'Bearer ' + key, 'Content-Type': 'application/json'})
    try:
        with urlopen(request, timeout=40) as response:
            result = json.load(response)
    except HTTPError as error:
        # 不回显服务端原始响应，避免日志意外包含敏感字段。
        hints = {401: '密钥无效', 403: '无访问权限', 404: '模型或接口不存在',
                 429: '额度不足或请求过于频繁'}
        raise ValueError(f'模型 API HTTP {error.code}：{hints.get(error.code, "服务请求失败")}。') from None
    except (URLError, TimeoutError):
        raise ValueError('模型 API 连接失败或超时，请检查网络和地址。') from None
    try:
        return result['choices'][0]['message']['content']
    except (KeyError, IndexError, TypeError):
        raise ValueError('模型 API 响应缺少 choices[0].message.content。') from None
