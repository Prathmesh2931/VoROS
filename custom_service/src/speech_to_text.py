#!/usr/bin/env python3

# import speech_recognition as sr

# # Create a recognizer instance
# recognizer = sr.Recognizer()

# # List all audio input devices
# print("Available audio devices:")
# for index, name in enumerate(sr.Microphone.list_microphone_names()):
#     print(f"{index}: {name}")

# # Select your USB headphone by its index
# usb_headphone_index = int(input("Enter the index of your USB headphone: "))

# # Use the selected microphone
# try:
#     with sr.Microphone(device_index=usb_headphone_index) as source:
#         print("Adjusting for ambient noise... Please wait.")
#         recognizer.adjust_for_ambient_noise(source)
#         print("Ready to recognize speech. Speak now!")

#         # Listen to the audio
#         audio = recognizer.listen(source)

#         # Convert speech to text
#         try:
#             text = recognizer.recognize_google(audio)
#             print("You said:", text)
#         except sr.UnknownValueError:
#             print("Sorry, could not understand the audio.")
#         except sr.RequestError as e:
#             print(f"Could not request results from Google Speech Recognition service; {e}")

# except Exception as e:
#     print(f"An error occurred: {e}")


import speech_recognition as sr

# Create a recognizer instance
recognizer = sr.Recognizer()

# List all audio input devices
print("Available audio devices:")
for index, name in enumerate(sr.Microphone.list_microphone_names()):
    print(f"{index}: {name}")

# Select your USB headphone by its index
usb_headphone_index = int(input("Enter the index of your USB headphone: "))

# Use the selected microphone
try:
    with sr.Microphone(device_index=usb_headphone_index) as source:
        print("Adjusting for ambient noise... Please wait.")
        recognizer.adjust_for_ambient_noise(source, duration=0.5)  # Faster adjustment

        print("Ready to recognize speech. Speak now!")
        print("Press Ctrl+C to exit.")

        while True:
            try:
                print("Listening...")
                # Listen to audio with shorter timeout and energy threshold
                recognizer.energy_threshold = 200  # Adjust sensitivity to environment
                recognizer.pause_threshold = 0.5  # Faster end-of-speech detection
                audio = recognizer.listen(source, timeout=3, phrase_time_limit=5)

                # Convert speech to text
                text = recognizer.recognize_google(audio)
                print("You said:", text)

            except sr.WaitTimeoutError:
                print("No speech detected, listening again...")
            except sr.UnknownValueError:
                print("Sorry, could not understand the audio.")
            except sr.RequestError as e:
                print(f"Could not request results from Google Speech Recognition service; {e}")
            except KeyboardInterrupt:
                print("\nExiting...")
                break

except Exception as e:
    print(f"An error occurred: {e}")
