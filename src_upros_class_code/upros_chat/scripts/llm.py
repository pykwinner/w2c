#!/usr/bin/env python3
from openai import OpenAI
import rospy
from std_msgs.msg import String

api_key = "sk-aTDemqxcD49oRFND5oeoSRbVaQSNtt10eRUgWTDiQ4xBsZZn"
base_url = "https://api.moonshot.cn/v1"

class LLM(OpenAI):
    def __init__(self):

        super().__init__(api_key=api_key,
                         base_url=base_url)
        
        self.model = "moonshot-v1-8k"
        self.system_role_content = "你是Kimi,由 Moonshot AI 提供的人工智能助手, \
        我们将会叫你的小名“小月”,你不会在你的回答中提及你的小名,你更擅长中文和英文的对话. \
        你会为用户提供安全,有帮助,准确的回答. \
        同时,你会拒绝一切涉及恐怖主义,种族歧视,黄色暴力等问题的回答"
        # 初始化ros节点
        rospy.init_node('robot_voice_llm_node', anonymous=True)
        rospy.Subscriber("/robot_voice/speech/result", String, self.speech_result_callback)
        self.llm_pub = rospy.Publisher('/robot_voice/llm/result',String, queue_size = 10)

    def speech_result_callback(self, msg):
        result = msg.data
        print("speech [{}]".format(result))
        llm_msg = String()
        if result:
            try:
                chat_response = self.query(result)
                indented_response = "\n".join(f"\t{line}" for line in chat_response.splitlines())
                print(f"LLM的返回结果: \n\n'''\n{indented_response}\n'''")
                if indented_response:
                    llm_msg.data = indented_response
                    self.llm_pub.publish(llm_msg)
            except Exception as e:
                if "rate_limit_reached" in str(e):  # 检查是否为RateLimitError
                    llm_msg.data = "请求超限"
                    self.llm_pub.publish(llm_msg)
                else:
                    llm_msg.data = "出错啦"
                    self.llm_pub.publish(llm_msg)
    
    def get_system_role_prompt(self):
        return {"role":"system",  "content": self.system_role_content}

    def user_prompt(self, user_prompt):
        return {"role": "user", "content": user_prompt}

    def query(self, user_prompt):
        user_message = [self.get_system_role_prompt(), self.user_prompt(user_prompt)]
        completion = self.chat.completions.create(
            model=self.model,
            messages=user_message,
            temperature=0.1,
            stream=False
        )
        return completion.choices[0].message.content

if __name__ == "__main__":

    try:
        llm = LLM()
        rospy.spin()
        r = rospy.Rate(0.2)
        r.sleep()
    except KeyboardInterrupt:
        print("\nCaught Ctrl + C. Exiting")
    
    