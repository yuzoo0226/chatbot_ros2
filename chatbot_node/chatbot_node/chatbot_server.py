#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import time
import openai

# ros messages
# from tmc_msgs.msg import Voice
# from tmc_msgs.msg import TalkRequestAction
# from tmc_msgs.msg import TalkRequestGoal
# from std_srvs.srv import Empty, EmptyResponse
# from std_msgs.msg import String

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from chatbot_msgs.action import Chat


class ChatbotServer(Node):
    """
    GPTを用いた返答文作成サーバ
    """

    def __init__(self) -> None:
        """
        マイクの設定とrosインタフェースの初期化
        """
        super().__init__("chatbot_server")

        self.declare_parameter("/response_server/language", "en")
        self.declare_parameter("/response_server/gpt_model", "gpt-4")
        self.declare_parameter("/response_server/robot", "HSR")

        self.language = self.get_parameter("/response_server/language").value
        self.gpt_model = self.get_parameter("/response_server/gpt_model").value
        self.robot_model = self.get_parameter("/response_server/robot").value

        self.openai_key = os.environ.get("OPENAI_API")
        openai.api_key = self.openai_key
        self.check_authenticate()

        # ros interface
        self._chat_action_server = ActionServer(self, Chat, 'chatbot_action', self.cb_make_response)
        # self.action_server_register("make_response", "tam_make_response", MakeResponseAction, self.cb_make_response)

        if self.robot_model == "HSR":
            pass
            # self.srv_chat_reset = rospy.Service('/tam_make_response/chat_history/reset', Empty, self.chat_history_reset)
            # self._ac_talk_request = actionlib.SimpleActionClient("/talk_request_action", TalkRequestAction)

        self.default_chat = [
            {"role": "system", "content": "あなたはレストランで働くサービスロボットです．"},
            {"role": "system", "content": "会話が続くようなフレンドリーな返答文を作成してください．一人称は「ぼく」，語尾は「のだ」「なのだ」でお願いします．"},
            {"role": "system", "content": "英語で話しかけられた場合は，英語で返答の返答文を作成してください．"},
            {"role": "system", "content": "一文目は「承知したのだ。」や「よくわからなかったのだ。」のように，自分の理解度のみを伝える簡潔な文にしてください．"},
            {"role": "system", "content": "指示文が来た場合は断らず，可能である旨を伝えてください．"},
            {"role": "system", "content": "商品名だけを告げられた場合は，その商品を注文されたと考えて返答文を作成してください．"},
            {"role": "user", "content": "あなたの名前はなんですか？製作者は誰ですか？"},
            {"role": "assistant", "content": "ぼくの名前はタムタムGPTなのだ．田向研究室の学生によって開発されたのだ．"},
        ]
        self.chat_history = self.default_chat.copy()
        self.get_logger().info("ready to openai make response server")

    def check_authenticate(self) -> bool:
        """OpenAIのAPIキーが認証されているかを検証する関数

        Returns:
            bool: 認証が通ればTrue, 通らなければFalse
        """
        try:
            _ = openai.models.list()
        except openai.AuthenticationError:
            self.get_logger().error("OPENAI_API authenticate is failed")
            return False
        except Exception as e:
            self.get_logger().error(f"{e}")
            return False
        self._logger.info("OPENAI API authenticate is success")
        return True

    def say_action(self, text: str, language="ja") -> None:
        if self.robot_model == "HSR":
            try:
                if language == "ja":
                    self.language = TextToSpeech.JAPANESE
                elif language == "en":
                    self.language = TextToSpeech.ENGLISH
                else:
                    msg = "Unknown language. You can use 'ja' or 'en'."
                    raise exceptions.InvalidLanguageError(msg)

                self.get_logger().debug(f"<{self.robot_model}>: {text}")
                goal = TalkRequestGoal()
                goal.data.interrupting = False
                goal.data.queueing = True
                goal.data.language = self.language
                goal.data.sentence = text
                self._ac_talk_request.send_goal(goal)
            except Exception as e:
                self.get_logger().warn(f"You choiced [{self.robot_model}], but I cannot connect talk request.")
                self.get_logger().warn(f"{e}")
                self.get_logger().info(f"HSR say: [{text}]")
        else:
            self.get_logger().info(f"{self.robot_model} say: [{text}]")

        return

    def delete(self) -> None:
        return

    def load_chat_settings(self, path: str) -> list:
        return []

    def chat_history_reset(self, req) -> bool:
        """
        サービスコールによって実行
        """
        self.chat_history = self.default_chat.copy()
        self.get_logger().info("reset chat history")
        return EmptyResponse()

    def cb_make_response(self, goal_handle: Chat.Goal) -> Chat.Result:
        """
        アクションリクエストが来たときのコールバック関数
        """
        prompt = {"role": "user", "content": goal_handle.request.prompt}
        language = "ja"
        is_stream = False
        is_say_last = True

        if goal_handle.request.stream is not None:
            is_stream = goal_handle.request.stream
        if goal_handle.request.say_last is not None:
            is_say_last = goal_handle.request.say_last
        if goal_handle.request.language == "en":
            language = goal_handle.request.language

        # 追加のプロンプトを入力
        if goal_handle.request.system_prompt is not None:
            self.chat_history.append({"role": "system", "content": f"次の出力においてのみ，{goal_handle.request.system_prompt}"})

        self.chat_history.append(prompt)

        response = openai.chat.completions.create(
            model=self.gpt_model,
            messages=self.chat_history,
            stream=is_stream
        )

        if is_stream:
            complete_messages = []
            collected_messages = []
            result_message = ""
            say_message = ""
            for chunk in response:
                chunk_message = chunk.choices[0].delta.content

                # 一文の完成 + それが最後の文ではないときに読み上げ（直前のループにて文が出来上がっていれば読み上げ）
                if say_message != "" and chunk_message != "":
                    try:
                        self._ac_talk_request.wait_for_result()
                        time.sleep(0.5)
                    except Exception as e:
                        self.get_logger().debug(f"{e}")
                        self.get_logger().debug("cannot wait action goal")
                    collected_messages = []
                    result_message = result_message + say_message
                    self.say_action(say_message, language)
                    say_message = ""

                # 一文の完成 + それが最後の文のときのよみあげ挙動は分岐
                elif say_message != "" and chunk_message == "":
                    if is_say_last:
                        try:
                            self._ac_talk_request.wait_for_result()
                            time.sleep(0.5)
                        except Exception as e:
                            self.get_logger().debug(f"{e}")
                            self.get_logger().debug("cannot wait action goal")

                        self.get_logger().debug("This is the last_message")
                        collected_messages = []
                        result_message = result_message + say_message
                        self.say_action(say_message, language)
                        say_message = ""
                    else:
                        try:
                            self._ac_talk_request.wait_for_result()
                            time.sleep(0.5)
                        except Exception as e:
                            self.get_logger().debug(f"{e}")
                            self.get_logger().debug("cannot wait action goal")

                        self.get_logger().debug("return last say message")
                        self.get_logger().debug(f"<{self.robot_model}>: {say_message}")

                collected_messages.append(chunk_message)
                complete_messages.append(chunk_message)
                if chunk_message in ["。", "、", "！", "？", ".", ",", "!", "?", ]:
                    say_message = "".join(collected_messages)

            complete_message = "".join([msg for msg in complete_messages if msg is not None])

            # Delete temporal system prompt
            if goal_handle.request.system_prompt is not None:
                self.chat_history.pop(-2)  # system_pormptは末尾から2番目

            self.chat_history.append({"role": "assistant", "content": complete_message})

            if is_say_last:
                goal_handle.succeed()
                result = Chat.Result()
                result.result = complete_message
            else:
                goal_handle.succeed()
                result = Chat.Result()
                result.result = say_message
        else:
            self.get_logger().info(f"response: {response.choices[0].message.content}")
            # Delete temporal system prompt
            if goal_handle.request.system_prompt is not None:
                self.chat_history.pop(-2)  # system_pormptは末尾から2番目

            self.chat_history.append({"role": "assistant", "content": response.choices[0].message.content})

            result = Chat.Result()
            result.result = response.choices[0].message.content
            goal_handle.succeed()

        return result

    def unit_test(self, is_stream=True) -> str:
        self.get_logger().info("質問を入力するのだ．qを入力すれば終了なのだ．")
        while True:
            user = input("<あなた>: ")
            if user == "q":
                break

            prompt = {"role": "user", "content": user}
            self.chat_history.append(prompt)

            response = openai.ChatCompletion.create(
                model=self.gpt_model,
                messages=self.chat_history,
                stream=True
            )

            if is_stream:
                complete_messages = []
                collected_messages = []
                say_message = ""
                for chunk in response:
                    chunk_message = chunk.choices[0].delta.content

                    # 一文の完成 + それが最後の文ではないときに読み上げ（直前のループにて文が出来上がっていれば読み上げ）
                    if say_message != "" and chunk_message != "":
                        collected_messages = []
                        print(f"<{self.robot_model}>: ", say_message)
                        say_message = ""

                    # 一文の完成 + それが最後の文のときのよみあげ挙動は分岐処理
                    elif say_message != "" and chunk_message == "":
                        if True:
                            print("This is the last_message")
                            collected_messages = []
                            print(f"<{self.robot_model}>: ", say_message)
                            say_message = ""
                        else:
                            self.get_logger().debug("return last say message")

                    collected_messages.append(chunk_message)
                    complete_messages.append(chunk_message)
                    # 文が完成したかをチェック
                    if chunk_message in ["。", "、", "！", "？", ".", ",", "!", "?", ]:
                        say_message = "".join(collected_messages)

                complete_message = "".join(complete_messages)
                self.chat_history.append({"role": "assistant", "content": complete_message})

            else:
                print(f"<{self.robot_model}>: ", response.choices[0].message.content)
                self.chat_history.append({"role": "assistant", "content": response.choices[0].message.content})


def main():
    rclpy.init()
    node = ChatbotServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
