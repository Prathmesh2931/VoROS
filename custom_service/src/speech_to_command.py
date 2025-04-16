#!/usr/bin/env python3

# import os
# import sounddevice as sd
# import soundfile as sf
# import subprocess
# from threading import Thread
# from dotenv import load_dotenv
# from groq import Groq

# load_dotenv()
# client = Groq(api_key=os.environ.get("GROQ_API_KEY"))

# speech_to_command_dict = {
#     "redeye takeoff": '''ros2 service call /takeoff std_srvs/srv/SetBool "{data: true}"''',
#     "redeye land": '''ros2 service call /takeoff std_srvs/srv/SetBool "{data: false}"''',
# }

# def clean_text(text):
#     """
#     Cleans and normalizes the text input.
#     """
#     return text.lower().strip().replace('.', '').replace(',', '')

# def record_audio(duration=5, sample_rate=16000, output_file="/tmp/input_audio.wav"):
#     """
#     Records audio from the microphone for a given duration.
#     """
#     print("Speak into the microphone.")
#     print(f"You have {duration} seconds to speak...")
#     audio = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=1, dtype="int16")
#     sd.wait()
#     sf.write(output_file, audio, sample_rate)
#     return output_file

# def transcribe_audio(file_path):
#     """
#     Transcribes the recorded audio using Groq's Whisper model.
#     """
#     with open(file_path, "rb") as audio_file:
#         transcription = client.audio.transcriptions.create(
#             file=(file_path, audio_file.read()),
#             model="whisper-large-v3-turbo",
#             language="en",
#         )
#     return transcription.text

# def parse_command(command):
#     """
#     Maps a spoken command to the corresponding ROS2 command using sentiment analysis and intent understanding.
#     """
#     chat_completion = client.chat.completions.create(
#         messages=[
#             {
#                 "role": "system",
#                 "content": f"""You are a sentiment and intent analyzer for ROS2 commands. Your job is to understand 
#                 the sentiment and context of spoken commands and map them to the appropriate ROS2 command. 
                
#                 Use the following dictionary for mapping intents to commands:
#                 {speech_to_command_dict}
                
#                 If the spoken command implies intent or sentiment for a specific action (e.g., 'lift up , takeoff' implies 'redeye takeoff'),
#                 match it to the corresponding key in the dictionary. If no match is possible, return 'INVALID'. 
#                 DO NOT PRINT ANYTHING EXCEPT THE KEY."""
#             },
#             {
#                 "role": "user",
#                 "content": f"Determine the command intent for: '{command}'"
#             }
#         ],
#         model="llama3-8b-8192",
#     )

#     parsed = chat_completion.choices[0].message.content.strip().strip("'").strip('"')
#     # print(f"LLM parsed result: {parsed}") 

#     return speech_to_command_dict.get(parsed, None)

# def execute_command(command):
#     """
#     Executes the given command-line ROS2 command asynchronously.
#     """
#     try:
#         process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
#         print(f"Command is running in the background: {command}")
#         Thread(target=log_command_output, args=(process,)).start()
#     except Exception as e:
#         print(f"Error executing command: {e}")

# def log_command_output(process):
#     """
#     Logs the output of the command asynchronously.
#     """
#     stdout, stderr = process.communicate()
#     if stdout:
#         print("Command Output:", stdout.decode())
#     if stderr:
#         print("Command Error:", stderr.decode())

# if __name__ == "__main__":
#     try:
#         while True:
#             audio_file = record_audio()
#             transcript = transcribe_audio(audio_file)
#             transcript = clean_text(transcript)  
#             print(f"\nNormalized Transcription: {transcript}")

#             parsed_command = parse_command(transcript)
#             if parsed_command:
#                 print(f"Executing Command: {parsed_command}")
#                 execute_command(parsed_command)
#             else:
#                 print("Invalid or unrecognized command.")
#     except KeyboardInterrupt:
#         print("\nExiting gracefully...")
#     except Exception as e:
#         print(f"Unexpected Error: {e}")

#!/usr/bin/env python3

import os
import sounddevice as sd
import soundfile as sf
import subprocess
from threading import Thread
from dotenv import load_dotenv
from groq import Groq
import re

load_dotenv()
# api_key = os.environ.get("GROQ_API_KEY")
# if not api_key:
api_key = "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"

client = Groq(api_key=api_key)

# client = Groq(api_key=os.environ.get("GROQ_API_KEY"))

speech_to_command_dict = {
    "forward": '''ros2 service call /text_move custom_service/srv/Text "{input: forward}"''',
    "back": '''ros2 service call /text_move custom_service/srv/Text "{input: back}"''',
    "left": '''ros2 service call /text_move custom_service/srv/Text "{input: left}"''',
    "right": '''ros2 service call /text_move custom_service/srv/Text "{input: right}"''',
    "stop": '''ros2 service call /text_move custom_service/srv/Text "{input: stop}"''',
    # Dynamic commands with degrees
    # "turn_left_degrees": '''ros2 service call /text_move custom_service/srv/Text "{{input: turn_left_{deg}}}"''',
    # "turn_right_degrees": '''ros2 service call /text_move custom_service/srv/Text "{{input: turn_right_{deg}}}"''',
    "turn_degrees": '''ros2 service call /text_move custom_service/srv/Text "{{input: turn_{deg}}}"''',

}

def clean_text(text):
    """
    Cleans and normalizes the text input.
    """
    return text.lower().strip().replace('.', '').replace(',', '')

def record_audio(duration=3, sample_rate=16000, output_file="/tmp/input_audio.wav"):
    """
    Records audio from the microphone for a given duration.
    """
    print("Speak into the microphone.")
    print(f"You have {duration} seconds to speak...")
    audio = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=1, dtype="int16")
    sd.wait()
    sf.write(output_file, audio, sample_rate)
    return output_file

def transcribe_audio(file_path):
    """
    Transcribes the recorded audio using Groq's Whisper model.
    """
    with open(file_path, "rb") as audio_file:
        transcription = client.audio.transcriptions.create(
            file=(file_path, audio_file.read()),
            model="whisper-large-v3-turbo",
            language="en",
        )
    return transcription.text

# def parse_command(command):
#     """
#     Maps a spoken command to the corresponding ROS2 command using sentiment analysis and intent understanding.
#     """
#     chat_completion = client.chat.completions.create(
#         messages=[
#             {
#                 "role": "system",
#                 "content": f"""You are a sentiment and intent analyzer for ROS2 commands. Your job is to understand 
#                 the sentiment and context of spoken commands and map them to the appropriate ROS2 command. 
                
#                 Use the following dictionary for mapping intents to commands:
#                 {speech_to_command_dict}
                
#                 If the spoken command implies intent or sentiment for a specific action (e.g., 'lift up , takeoff' implies 'redeye takeoff'),
#                 match it to the corresponding key in the dictionary. If no match is possible, return 'INVALID'. 
#                 DO NOT PRINT ANYTHING EXCEPT THE KEY."""
#             },
#             {
#                 "role": "user",
#                 "content": f"Determine the command intent for: '{command}'"
#             }
#         ],
#         model="llama3-8b-8192",
#     )

#     parsed = chat_completion.choices[0].message.content.strip().strip("'").strip('"')
#     # print(f"LLM parsed result: {parsed}") 

#     return speech_to_command_dict.get(parsed, None)

import re
def parse_command(command):
    """
    Parses input like 'turn 90 degrees', 'turn 180', etc.
    Returns the full ROS2 service command string or None.
    """
    match = re.search(r'\bturn\s+(?:left|right)?\s*(\d{1,3})\s*(?:degrees|degree)?', command, re.IGNORECASE)
    if match:
        degree = match.group(1)
        return speech_to_command_dict["turn_degrees"].format(deg=degree)

    # Fallback to basic direction command
    chat_completion = client.chat.completions.create(
        messages=[
            {
                "role": "system",
                "content": f"""You are a command parser for ROS2. Match the spoken command to a key in this dictionary:
                {list(speech_to_command_dict.keys())}.
                Return only the key, or 'INVALID' if it doesn't match."""
            },
            {
                "role": "user",
                "content": f"{command}"
            }
        ],
        model="llama3-8b-8192",
    )

    parsed_key = chat_completion.choices[0].message.content.strip().strip('"').strip("'")
    if parsed_key in speech_to_command_dict:
        return speech_to_command_dict[parsed_key]

    return None


import re

# def parse_command(command):
#     """
#     Parses input like 'turn 90 degrees', 'turn left 180', etc.
#     Returns just 'turn_{degree}' (e.g., 'turn_90').
#     """
#     match = re.search(r'\bturn\s+(?:left|right)?\s*(\d{1,3})\s*(?:degrees|degree)?', command, re.IGNORECASE)
#     if match:
#         degree = match.group(1)
#         return f"turn_{degree}"

#     return None



def execute_command(command):
    """
    Executes the given command-line ROS2 command asynchronously.
    """
    try:
        process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print(f"Command is running in the background: {command}")
        Thread(target=log_command_output, args=(process,)).start()
    except Exception as e:
        print(f"Error executing command: {e}")

def log_command_output(process):
    """
    Logs the output of the command asynchronously.
    """
    stdout, stderr = process.communicate()
    if stdout:
        print("Command Output:", stdout.decode())
    if stderr:
        print("Command Error:", stderr.decode())

if __name__ == "__main__":
    try:
        while True:
            audio_file = record_audio()
            transcript = transcribe_audio(audio_file)
            transcript = clean_text(transcript)  
            # print(f"\nNormalized Transcription: {transcript}")

            parsed_command = parse_command(transcript)
            if parsed_command:
                # print(f"Executing Command: {parsed_command}")
                execute_command(parsed_command)
            else:
                print("Invalid or unrecognized command.")
    except KeyboardInterrupt:
        print("\nExiting gracefully...")
    except Exception as e:
        print(f"Unexpected Error: {e}")
