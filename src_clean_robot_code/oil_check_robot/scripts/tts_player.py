import sherpa_onnx
import queue
import time
import soundfile as sf
import subprocess
from pathlib import Path


class TTS():
    def __init__(self, 
                model_dir=None, sid=100, debug=False, num_threads=2):
        if model_dir is None:
            print("模型路径空缺,请指定模型路径")
            exit(1)
        self.model_dir = model_dir
        self.model = self.model_dir + "/vits-zh-hf-fanchen-C.onnx"
        self.dict_dir = self.model_dir + "/dict"
        self.lexicon = self.model_dir + "/lexicon.txt"
        self.tokens = self.model_dir + "/tokens.txt"
        self.tts_rule_fsts = self.model_dir + "/number.fst"
        # sid 是说话人的 id，用于区分不同的说话人, 其取值范围是0-108
        self.sid = 0 if sid > 108 else sid
        self.num_threads = num_threads
        self.debug = debug
        self.output_filename = "genrerated.wav"
        self.buffer = queue.Queue()
        self.started = False
        self.stopped = False
        self.killed = False
        self.sample_rate = None
        self.first_message_time = None
        self.tts_config = self.generate_config()
        self.load_model()

    """加载TTS模型并初始化"""    
    def load_model(self):
        if not self.tts_config.validate():
            raise ValueError("Please check your config")
        print("载入模型中 ...")
        self.tts = sherpa_onnx.OfflineTts(self.tts_config)
        self.sample_rate = self.tts.sample_rate
        print("模型载入完成.")

    def _assert_file_existence(self, filepath: str) -> None:
        assert Path(filepath).is_file(), (
            f"模型 {filepath} 不存在!\n"    
        )

    def _assert_dir_existence(self, dirpath: str) -> None:
        assert Path(dirpath).is_dir(), (
            f"目录 {dirpath} 不存在!\n"    
         )

    def generate_config(self):
        self._assert_file_existence(self.model)
        self._assert_file_existence(self.lexicon)
        self._assert_dir_existence(self.dict_dir)
        self._assert_file_existence(self.tokens)
        tts_config = sherpa_onnx.OfflineTtsConfig(
            model=sherpa_onnx.OfflineTtsModelConfig(
                vits=sherpa_onnx.OfflineTtsVitsModelConfig(
                    model=self.model,
                    lexicon=self.lexicon,
                    dict_dir=self.dict_dir,
                    tokens=self.tokens,
                ),
                provider="cpu",
                debug=self.debug,
                num_threads=self.num_threads,
            ),
            rule_fsts=self.tts_rule_fsts,
        )
        return tts_config

    def reset_flags_and_event(self):
        self.started = False
        self.stopped = False
        self.killed = False

    def validate_args(self):
        if not self.model:
            raise ValueError("--vits-model must be provided")

    def generate_and_play(self, text):
        self.reset_flags_and_event()
        if not self.tts_config.validate():
            raise ValueError("请检查你的配置文件")
        
        print("开始生成语音 ...")
        
        audio = self.tts.generate(text, sid = 100, speed = 1.0)
        
        print("完成语音生成!")
        self.stopped = True

        if len(audio.samples) == 0:
            print("语音生成错误")
            self.killed = True
            return
        
        # 保存生成的语音文件到指定位置
        sf.write(self.output_filename,audio.samples,samplerate=audio.sample_rate,subtype="PCM_16")
        
        print("保存文件完成")
        
        time.sleep(1.0)
        
        subprocess.run(["aplay", self.output_filename])
        
        
