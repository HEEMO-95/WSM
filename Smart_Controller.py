import State_client
import Hawk_Vision
import threading

threading.Thread(target=State_client.program,daemon=False).start()  # start state_client program, server_handlin will start and close automatically
threading.Thread(target=Hawk_Vision.program,daemon=True).start()    # start Hawk_Vision program, to get YOLO (x,y,type) readings

while True:
    if Hawk_Vision.x and Hawk_Vision.y and Hawk_Vision.type == True: # reading from yolo example
      State_client.pan_cmd = cos(Hawk_Vision.x)   # writing pan control command example
      State_client.tilt_cmd = sin(Hawk_Vision.y)  # writing tilt control command example
