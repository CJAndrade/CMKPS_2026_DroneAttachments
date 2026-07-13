## INTRODUCTION
Drone-based surveillance systems are widely used for monitoring restricted areas and improving security. This project uses a Raspberry Pi, camera module, OpenCV, and the YOLO algorithm to detect humansin real time.
## EQUIPMENT
<img width="1200" height="1600" alt="WhatsApp Image 2026-06-23 at 14 43 06" src="https://github.com/user-attachments/assets/be08a059-0551-4602-9154-225988e4fd3c" />

### HARDWARE

1.Raspberry pi

2.Camera Module  

3.Brushless Motors 

4.pixhawk 2.4.8

### SOFTWARE

1.Raspberry Pi OS  

2.Python	

3.OpenCV 	

4.YOLO (you look only once)

5.Fusion 360 	

5.HTML (web monitoring app)
##METHODOLOGY
1.SD card flashing - Raspberry pi 0s written onto SD card to reboot the Raspberry pi 
2.Drone development: The drone is deployed over the surveillance area and follows a predefined flight path. The Pixhawk flight controller is responsible for maintaining flight stability,navigation, and communication with the Raspberry Pi.

3. Image Acquisition: live video stream captured using camera module

4.Video Frame Processing: The live video stream is divided into individual frames. OpenCV is used to process these frames by resizing, filtering, and converting them into a format suitable for object detection. 

5.Human Detection Using YOLO :
The processed frames are provided as input to the YOLO (You Only Look Once) object detection model. YOLO analyzes each frame and identifies the presence of humans. When a human is detected, the model generates a bounding box around the detected person along with a confidence score.

6.web monitoring app - created a web app using HTML for real time monitoring

## RESULTS 
Successfully detected humans from pre-recorded video footage
<img width="1280" height="960" alt="WhatsApp Image 2026-06-24 at 14 55 12" src="https://github.com/user-attachments/assets/c616bf2a-8268-474b-9b3f-0c4871c0a04a" />
<img width="1280" height="960" alt="WhatsApp Image 2026-06-24 at 14 55 49" src="https://github.com/user-attachments/assets/89b057e8-7057-4189-95c6-c1d63f647c55" />

Successfully detected humans from live video stream
<img width="929" height="1600" alt="WhatsApp Image 2026-07-13 at 15 39 28" src="https://github.com/user-attachments/assets/159e2951-9d65-40e4-84eb-4cc9be6754f6" />
YOLO model generates bounding box around detected humans with confidence score

## CONCLUSION

Successfully detected humans from both recorded and live video using YOLO object detection on a drone surveillance system built with Raspberry Pi and Pixhawk Flight Controller.



