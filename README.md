4/24/2025

# FER_DataCollection
Repo for code relating to facial expression recognition. Primary function is in ./modified_legacy_code/EmotionDetection.py for recognizing negative emotions and identifying incapacitation.

# DockerHub Repo
Listed below is the link to the private Dockerhub repo that stores the project images. You can also see the FER image in the docker-files repository under ./services/fer/Dockerfile (check test-fer-integration branch if you don't see it). READ THIS => all code repo's are local to the machine running the cart "jacart-project/dev_ws/src/*REPOS*". The "jacart-project/dev_ws" level just holds ros deps (install, ...). This local repo is bind-mounted into the container, and the container holds the other dependencies for code.
- [DockerHub](https://hub.docker.com/repository/docker/jmujacart/jacart_images/general) Storing project images.

# Required Dependencies
Disclaimer, the Dockerfile for this package contains all these things. What's listed below is mostly the pip
install requirements. Things like CUDA or ROS2 are taken care of in the Dockerfile.
- [FER](https://pypi.org/project/fer/): Primary package for identifying faces and emotions.
- [TensorFlow](https://pypi.org/project/tensorflow/): Dependency for FER.
- [ffmpeg](https://pypi.org/project/ffmpeg/): Dependency for FER.
- [moviepy](https://pypi.org/project/moviepy/): Dependency for FER.
- [ZED SDK](https://www.stereolabs.com/developers/release): The SDK install.
- [MySQL](https://pypi.org/project/mysql-connector-python/): For DB creation & interaction for Authentication scripts. DB creation is also needed, see https://github.com/ddomn1001/FaceRec_Emotion for DB setup instructions.
- [Argon2](https://pypi.org/project/argon2-cffi/): Used for hashing in Authentication scripts.
- [DotEnv](https://pypi.org/project/python-dotenv/): Used for loading environment variables. Primarily helps in development.

# ./legacy_code
This is original code from Dominic Nguyen from Spring 24'. This is not used for the cart, it is just the basis that everything else was developed on.

# ./modified_legacy_code
EmotionDetection, FacialRecognition (Auth), PostureRecognition (OoB & skeleton tracking)
openai_call is the point that EmotionDetection & Posture Recognition use when an alert is reached. They send the frame and a prompt to ChatGPT for a final verdict on an action.

# ./tools
Spare files for testing. These can be ignored unless you need the functionality of one of them. 