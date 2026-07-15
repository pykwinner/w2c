#!/usr/bin/env python3

import sys
import datetime
import json
import sherpa_onnx
import rospy
from std_msgs.msg import String
import sounddevice as sd
import wave
import pyaudio
import time

MODEL_ROOT = "/home/bcsh/Documents/Models/sherpa-onnx-streaming-zipformer-bilingual-zh-en-2023-02-20"
ENCODER = "encoder-epoch-99-avg-1.onnx"
DECODER = "decoder-epoch-99-avg-1.onnx"
JOINER = "joiner-epoch-99-avg-1.onnx"
TOKENS = "tokens.txt"

class SpeechRecognition:

    def __init__(self):
        # 初始化ros节点
        rospy.init_node('robot_voice_speech_node', anonymous=True)
        rospy.Subscriber("/robot_voice/speech/task", String, self.speech_callback)
        self.speech_result_pub = rospy.Publisher('/robot_voice/speech/result',String, queue_size = 10)
        self.speech_flag = True
        devices = sd.query_devices()
        if len(devices) == 0:
            print("No microphone devices found")
            sys.exit(0)

        print("Creating recognizer")
        self.recognizer = self.create_recognizer()
        print("Started! Please speak")
        self.stream = self.recognizer.create_stream()
        self.asr_result = "" 
        self.start()
    
    #收到'start'指令，speech_flag置为True
    def speech_callback(self, msg):
        if msg.data == "start":
            self.recognizer.reset(self.stream)
            self.asr_result = ""
            print("Continue Chatting....")
            self.speech_flag = True
           
    def start(self):
        sample_rate = 16000
        samples_per_read = int(0.1 * sample_rate)  # 0.1 second = 100 ms
        last_result = ""
        is_new_speech = True
        with sd.InputStream(channels=1, dtype="float32", samplerate=sample_rate) as s:
            while True:
                samples, _ = s.read(samples_per_read)  # 获取音频流
                samples = samples.reshape(-1) #展平成一维数组
                
                self.stream.accept_waveform(sample_rate, samples) #数据流管道输入

                while self.recognizer.is_ready(self.stream): #解码
                    self.recognizer.decode_stream(self.stream)
                         
                # 获取识别结果
                result = self.recognizer.get_result(self.stream)
                
                # 对比识别结果字符串，如果更新则替换
                if result and (last_result != result):
                    result_diff = result.replace(last_result, "")
                    last_result = result
                    if is_new_speech:
                        is_new_speech = False
                        timestamp = datetime.datetime.now()
                    print(result)
                
                self.asr_result = result
                
                # 到了语音音频的结束点
                is_endpoint = self.recognizer.is_endpoint(self.stream)
                
                # 语音停止输入了
                if is_endpoint:
                    if self.asr_result and self.speech_flag:
                        print(self.asr_result)
                        duration =  datetime.datetime.now() - timestamp
                        print("Recognize Duration: " + str(duration))
                        #识别结果通过topic的形式发布出去
                        msg = String()
                        msg.data = result
                        self.speech_result_pub.publish(msg)
                        self.speech_flag = False
                        is_new_speech = True
                    self.recognizer.reset(self.stream)

    #根据本地模型初始化识别器
    def create_recognizer(self):
        recognizer = sherpa_onnx.OnlineRecognizer.from_transducer(
            tokens=MODEL_ROOT + "/" + TOKENS,
            encoder=MODEL_ROOT + "/" + ENCODER,
            decoder=MODEL_ROOT + "/" + DECODER,
            joiner=MODEL_ROOT + "/" + JOINER,
            num_threads=1,
            sample_rate=16000,
            feature_dim=80,
            enable_endpoint_detection=True,
            rule1_min_trailing_silence=2.4,
            rule2_min_trailing_silence=1.2,
            rule3_min_utterance_length=300,  # it essentially disables this rule
            decoding_method="greedy_search",
            provider="cpu",
            hotwords_file="",
            hotwords_score=1.5,
            blank_penalty=0.0,
        )
        return recognizer
     
if __name__ == "__main__":
    try:
        speech = SpeechRecognition()
        rospy.spin()
        r = rospy.Rate(0.2)
        r.sleep()
    except KeyboardInterrupt:
        print("\nCaught Ctrl + C. Exiting")
