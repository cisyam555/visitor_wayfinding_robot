import rclpy
from rclpy.node import Node

from interaction_msgs.srv import TtsService

class ChatSay(Node):
    def __init__(self) -> None:
        super().__init__('chat_say')
        self.declare_parameter('text', '你好，我是导览机器人小静')
        self.declare_parameter('audio_id', 'audio_guide')
        self.declare_parameter('cmd', 'append')
        self._client = self.create_client(
            TtsService, '/intelligent_interaction/tts/play')

    def say(self, text: str) -> bool:
        if not self._client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('TTS 服务未就绪，请先 source ~/xos/setup.bash')
            return False
        request = TtsService.Request()
        request.cmd = str(self.get_parameter('cmd').value)
        request.type = 'text'
        request.text = text
        request.audio_id = str(self.get_parameter('audio_id').value)
        request.extra = ''
        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done():
            self.get_logger().error('TTS 调用超时')
            return False
        response = future.result()
        ok = bool(response is not None and response.success)
        self.get_logger().info(
            f'播报「{text}」：{"成功" if ok else "失败"} '
            f'status={getattr(response, "status", "")}')
        return ok

def main() -> None:
    rclpy.init()
    node = ChatSay()
    try:
        node.say(str(node.get_parameter('text').value))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()