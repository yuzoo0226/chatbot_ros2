#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from chatbot_msgs.action import Chat


class ChatbotClientUtils(Node):

    def __init__(self) -> None:
        super().__init__("chatbot_client_utils")
        self._action_client = ActionClient(self, Chat, 'chatbot_action')
        self.is_get_reult = False

    def _send_goal(self, prompt: str, language: str, system_prompt: str, is_stream: bool, is_say_last: bool):
        goal_msg = Chat.Goal()
        goal_msg.language = language
        goal_msg.prompt = prompt
        goal_msg.system_prompt = system_prompt
        goal_msg.stream = is_stream
        goal_msg.say_last = is_say_last

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: {feedback_msg.feedback.status_message} (Current distance: {feedback_msg.feedback.current_distance})')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.result}')
        self.result = result.result

    def chat_request(self, prompt: str, language: str, system_prompt: str = "", is_stream=False, is_say_last=True) -> str:
        """_summary_

        Args:
            prompt (str): 質問文
            language (str): 今回の会話においてのみ追加するシステムプロンプト
            system_prompt (str): 言語 ["ja" or "en"]
            is_stream (bool): リアルタイム発話
            is_say_last (bool): 最後の文を発話するかどうか

        Returns:
            str: _description_
        """
        self._send_goal(prompt, language, system_prompt, is_stream, is_say_last)


def main():
    rclpy.init()
    chatbot_client_utils = ChatbotClientUtils()
    chatbot_client_utils.chat_request("こんにちは.", "ja", "", True, True)
    rclpy.spin(chatbot_client_utils)


if __name__ == "__main__":
    main()
