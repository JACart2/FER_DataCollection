2/27/2025

# FER_DataCollection
Code repo for developing facial expression recognition for the AV (cart).

# legacy_code
This is original code from Dominic Nguyen from Spring 23'. This is not used for the cart, it is just the basis that everything else was developed on.

# modified_legacy_code
EmotionDetection, FacialRecognition (Auth), PostureRecognition (OoB & skeleton tracking)
openai_call is the point that EmotionDetection & Posture Recognition use when an alert is reached. They send the frame and a prompt to ChatGPT for a final verdict on an action.

# tools
Spare files for testing. These can be ignored unless you need the functionality of one of them. 
    get_python_pyzed.py installs pyzed on a Windows machine.