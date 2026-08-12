import socket
import cv2
from datetime import datetime
import time
import threading
import pyaudio
import wave
import subprocess
import os

# Based in example provided here:
# https://stackoverflow.com/questions/14140495/how-to-capture-a-video-and-audio-in-python-from-a-camera-or-webcam


class Video:
    def __init__(self):
        # Opening camera
        self.capture = cv2.VideoCapture(0)
        if not self.capture.isOpened():
            print("Cannot open camera.")
            exit()

        # Getting fps and image size from camera
        #self.fps = self.capture.get(cv2.CAP_PROP_FPS)
        self.fps = 5
        self.size = (int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
                     int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))

        self.frame_count = 1
        self.start_time = None
        self.file_name = None
        self.writer = None
        self.record = False
        self.show_image = False

    def start(self, show_image: bool = False, file_name: str = "temp_video"):
        if not self.record:
            self.record = True
            self.start_time = time.time()

            # Configuring video output
            self.file_name = file_name + ".mp4"
            vwf = cv2.VideoWriter_fourcc(*'mp4v')
            self.writer = cv2.VideoWriter(self.file_name, vwf, self.fps, self.size)
            self.show_image = show_image

            # Starting thread to record video
            self.video_thread = threading.Thread(target=self.recording)
            self.video_thread.start()

    def recording(self):
        while self.record:
            # Reading frame from camera
            ret, image = self.capture.read()

            if not ret:
                print("Cannot receive image.")
                break
            else:
                # Adding date and time to the image
                cv2.putText(image, datetime.now().strftime("%d.%m.%Y %H:%M:%S"),
                            (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (17, 190, 252), 2, cv2.LINE_AA)

                # Writing video
                self.writer.write(image)
                self.frame_count = self.frame_count + 1

                # Show image on screen
                if self.show_image:
                    cv2.imshow('image', image)

                time.sleep(1/self.fps)

    def stop(self):
        # Stopping current recording process
        if self.record:
            self.record = False

            # Waiting for thread to finish
            self.video_thread.join()

            self.writer.release()
            self.capture.release()
            cv2.destroyAllWindows()

            print("Video finished: " + self.file_name)


class Audio:
    def __init__(self):
        self.audio = pyaudio.PyAudio()
        self.rate = 44100  # 44100 samples per second
        self.frames_per_buffer = 4096  # record in chunks of 4096 samples
        self.channels = 1
        self.format = pyaudio.paInt16  # 16 bits per sample

        self.audio = pyaudio.PyAudio()
        self.stream = None

        self.start_time = None
        self.file_name = None
        self.audio_frames = []

        self.record = False
        self.audio_thread = None

    def start(self, file_name: str = "temp_audio"):
        if not self.record:
            self.record = True

            self.start_time = time.time()
            self.file_name = file_name + ".wav"

            self.stream = self.audio.open(format=self.format,
                                          channels=self.channels,
                                          rate=self.rate,
                                          input=True,
                                          input_device_index=0,
                                          frames_per_buffer=self.frames_per_buffer)

            # Starting thread to record video
            self.audio_thread = threading.Thread(target=self.recording)
            self.audio_thread.start()

    def recording(self):
        # Starting stream
        self.stream.start_stream()

        # Getting data
        while self.record:
            try:
                data = self.stream.read(self.frames_per_buffer)
                self.audio_frames.append(data)
            except Exception as e:
                print(e)
            time.sleep(0.01)

        # Stopping stream
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()

        # Saving audio file
        wave_file = wave.open(self.file_name, 'wb')
        wave_file.setnchannels(self.channels)
        wave_file.setsampwidth(self.audio.get_sample_size(self.format))
        wave_file.setframerate(self.rate)
        wave_file.writeframes(b''.join(self.audio_frames))
        wave_file.close()

    def stop(self):
        # Stopping current recording process
        if self.record:
            self.record = False

            # Waiting for thread to finish
            self.audio_thread.join()

            print("Audio finished: " + self.file_name)


class AudioVideo:
    def __init__(self):
        self.video = None
        self.audio = None
        self.file_name = None

    def start(self, file_name: str = "video_audio"):
        self.video = Video()
        self.audio = Audio()

        self.video.start()
        self.audio.start()
        self.file_name = file_name + ".mp4"

    def stop(self):
        self.audio.stop()

        # Calculating final FPS
        frame_counts = self.video.frame_count
        duration = time.time() - self.video.start_time
        fps = frame_counts / duration

        self.video.stop()

        # Waiting until all threads have finished
        while threading.active_count() > 1:
            print("...... Waiting threads to finish before merging ......")
            print(threading.active_count())
            time.sleep(0.5)

        # Starting thread to merge audio and video
        merge_thread = threading.Thread(target=self.merge,
                                        args=(fps, self.audio.file_name,
                                              self.video.file_name))
        merge_thread.start()

        self.audio = None
        self.video = None

    def merge(self, recorded_fps, audio_file_name, video_file_name):
        # Merging audio and video

        if abs(recorded_fps - 6) >= 0.01:
            # If the fps rate was higher/lower than expected, re-encode
            # it to the expected
            cmd = ("ffmpeg -r " + str(recorded_fps) + " -i " +
                   video_file_name +
                   " -pix_fmt yuv420p -r 6 temp_video2.mp4")
            subprocess.call(cmd, shell=True)

            cmd = ("ffmpeg -y -ac 2 -channel_layout mono -i " +
                   audio_file_name +
                   " -i temp_video2.mp4 -pix_fmt yuv420p " + self.file_name)
            subprocess.call(cmd, shell=True)
        else:
            cmd = ("ffmpeg -y -ac 2 -channel_layout mono -i " +
                   audio_file_name + " -i " + video_file_name +
                   " -pix_fmt yuv420p " + self.file_name)
            subprocess.call(cmd, shell=True)

        # Deleting temporary file
        self.delete_temp_files(audio_file_name, video_file_name)
        print("Video with audio finished: " + self.file_name)

    def delete_temp_files(self, audio_file_name, video_file_name):
        # Deleting temporary files
        local_path = os.getcwd()
        if os.path.exists(str(local_path) + "/" + audio_file_name):
            os.remove(str(local_path) + "/" + audio_file_name)
        if os.path.exists(str(local_path) + "/" + video_file_name):
            os.remove(str(local_path) + "/" + video_file_name)
        if os.path.exists(str(local_path) + "/temp_video2.mp4"):
            os.remove(str(local_path) + "/temp_video2.mp4")


def main():
    # Creating server
    socket_obj = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    socket_obj.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    socket_obj.bind(("", 2222))
    socket_obj.listen(2)
    #socket_obj.setblocking(False)

    print("Started record server")

    r = None

    while True:
        try:
            # Accepting and reading client request
            c, addr = socket_obj.accept()
            request = c.recv(64).decode('utf-8')
            print("Client request: " + request)

            if "start" in request:
                if r is None:
                    c.send("Start recording".encode())
                    if "audio" in request and "video" not in request:
                        # Start audio recording only
                        r = Audio()
                        r.start(file_name=datetime.now().strftime("%d-%m-%Y_%H.%M.%S"))
                    if "audio" not in request and "video" in request:
                        # Start video recording only
                        r = Video()
                        r.start(file_name=datetime.now().strftime("%d-%m-%Y_%H.%M.%S"))
                    if "audio" in request and "video" in request:
                        # Start audio and video recording
                        r = AudioVideo()
                        r.start(file_name=datetime.now().strftime("%d-%m-%Y_%H.%M.%S"))

            if 'stop' in request:
                # Stopping current recording process
                if r is not None:
                    message = "Stop recording: " + r.file_name
                    c.send(message.encode())
                    r.stop()
                r = None

            c.close()

        except KeyboardInterrupt:
            if r is not None:
                r.stop()

            while threading.active_count() > 1:
                print("...... Waiting threads to finish before exiting ......")
                print(threading.active_count())
                time.sleep(0.5)

            socket_obj.shutdown(socket.SHUT_RDWR)
            socket_obj.close()

            break
        except Exception:
            pass


if __name__ == "__main__":
    main()