2/27/2025

# FER_DataCollection
Code repo for developing facial expression recognition for the AV (cart).

# Required Dependencies
- [FER](https://pypi.org/project/fer/): Primary package for identifying faces and emotions.
- [TensorFlow](https://pypi.org/project/tensorflow/): Dependency for FER.
- [ffmpeg](https://pypi.org/project/ffmpeg/): Dependency for FER.
- [moviepy](https://pypi.org/project/moviepy/): Dependency for FER.
- [ZED SDK](https://www.stereolabs.com/developers/release): The SDK install.
- [PyZed API](https://github.com/stereolabs/zed-python-api): Dependencies: ZED SDK (see above), python3-dev, python3-pip, opencv-python, pyopengl, cython, numpy => Linux: script @ /usr/local/zed/
- [MySQL](https://pypi.org/project/mysql-connector-python/): For DB creation & interaction for Authentication scripts. DB creation is also needed, see https://github.com/ddomn1001/FaceRec_Emotion for DB setup instructions.
- [Argon2](https://pypi.org/project/argon2-cffi/): Used for hashing in Authentication scripts.
- [DotEnv](https://pypi.org/project/python-dotenv/): Used for loading environment variables. Primarily helps in development.

# ./legacy_code
This is original code from Dominic Nguyen from Spring 23'. This is not used for the cart, it is just the basis that everything else was developed on.

# ./modified_legacy_code
EmotionDetection, FacialRecognition (Auth), PostureRecognition (OoB & skeleton tracking)
openai_call is the point that EmotionDetection & Posture Recognition use when an alert is reached. They send the frame and a prompt to ChatGPT for a final verdict on an action.

# ./tools
Spare files for testing. These can be ignored unless you need the functionality of one of them. 
    get_python_pyzed.py installs pyzed on a Windows machine.