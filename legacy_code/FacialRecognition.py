# Script for Facial Recognition
# Dominic Nguyen
# Version 6
# Date: 04/09/2024
import cv2
import mysql.connector
from tkinter import Tk, Button, Label, Entry
import pickle
from deepface import DeepFace
from scipy.spatial.distance import cosine
from threading import Thread

    # OpenCV library (cv2): Used for various computer vision tasks including face detection and image processing.
    # cv2.CascadeClassifier(): Pre-trained Haar cascade classifier for face detection. Pre-trained XML files for detecting objects such as faces, eyes, and smiles based on Haar-like features.
    # 'haarcascade_frontalface_default.xml': XML file specifically for detecting frontal faces in images.
    # Trained to detect faces by analyzing patterns of intensity gradients in grayscale images. Identifies features like eyes, nose, and mouth to locate faces.
    # DeepFace library: Used for face recognition and facial attribute analysis.
    # model_name='Facenet': Model used for face embedding extraction. 'Facenet' is a deep learning model trained to generate embeddings from face images.
    # These embeddings are suitable for measuring similarity between faces using cosine similarity.
    # Ensures that a face is detected in the input image/frame before attempting to extract embeddings. Helps avoid errors when processing images without faces.
    # pickle library: Used for serializing and deserializing Python objects.
    # mysql.connector library: Used for interacting with MySQL databases.
    # tkinter library: Used for creating GUI

# Handles extracting face embeddings based on what is obtained from the camera capture
def extract_face_embeddings(frame):
    # Uses deepface/opencv library to detect faces.
    # Uses facenet model.
    result = DeepFace.represent(img_path=frame, model_name='Facenet', enforce_detection=True)
    face_embeddings = result[0]['embedding'] if isinstance(result, list) else result['embedding']
    return face_embeddings

# FOR FACES NOT YET IN THE DATABASE
# Encodes Faces in (GRAYSCALE >> HELPS WITH TOLERANCE AND RACE), inserting them into the database.
def insert_face_data(name, face_embeddings, preferred_speed):
    #CONNECTS to database with credentials
    try:
        # This is the database created on the new laptop. 
        # Develops connection to the database used to store/retrieve the facial encodings.
        # To create this database, refer to the README.md file. 
        db_connection = mysql.connector.connect(
            host="localhost",
            user="jacart",
            passwd="jacart2024",
            database="face_recognition_db"
        )
    
        cursor = db_connection.cursor()

        # Cross checks the database to ensure that duplicate faces are not found. 
        query_check_duplicates = "SELECT face_encoding FROM face_detect_live"
        cursor.execute(query_check_duplicates)
        results = cursor.fetchall()

        
        # Serialize the face embeddings using pickle.dumps() to store them in the database
        encoded_face_embeddings = pickle.dumps(face_embeddings)

        # Set the threshold for similarity score to consider a face already present in the database
        # Higher values indicate a stricter criterion for similarity
        # Lower values allow for more variation in face recognition
        emotion_tolerance = 0.6  


        # Loop through each result retrieved from the database
        for result in results:
            # Load the stored face data from the database
            db_face_embeddings = pickle.loads(result[0])

            # Compare the new face with the ones already in the database
            # Calculate how similar they are using a measure called cosine similarity
            similarity_score = 1 - cosine(face_embeddings, db_face_embeddings)

            # Check if the similarity score is higher than a set threshold
            if similarity_score > emotion_tolerance:
                # If it's similar enough, the face is likely already in the database
                # Print a message indicating that the face data already exists
                print(f"Data for {name} already exists in the database.")
                # Stop processing further since the data doesn't need to be added again
                return

                # If the face is not found to be similar to any in the database, proceed with adding it
                # Create a query to add the new face data to the database
                query_insert_data = "INSERT INTO face_detect_live (person_name, face_encoding, preferred_speed) VALUES (%s, %s, %s)"
                # Define the data to be added: person's name, face embeddings, and preferred speed
                data = (name, encoded_face_embeddings, preferred_speed)
                # Execute the query to add the data to the database
                cursor.execute(query_insert_data, data)
                # Save the changes made to the database
                db_connection.commit()
                # Print a message indicating that the data was added successfully
                print(f"Data for {name} inserted successfully.")

                # Close the cursor and database connection
                cursor.close()
                db_connection.close()

    # Handle any errors that might occur during database operations
    except mysql.connector.Error as error:
        # Print an error message describing the issue
        print("Error:", error)

#Creates a bounding box around head of the user (TO ENSURE THE USER IS IN FRAME)
def draw_bounding_box(frame, x, y, w, h, color):
    cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)

# Encodes the face before it is placed into the database
def encode_face():
    # Initialize the camera
    camera = cv2.VideoCapture(0)  

    # Load the pre-trained Haar cascade classifier for face detection
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

    # Define dimensions and position for a stationary box
    box_width, box_height = 200, 200
    screen_width, screen_height = 640, 480
    box_x = int((screen_width - box_width) / 2)
    box_y = int((screen_height - box_height) / 2)

    # Set tolerance for distance between face and box
    tolerance = 50

    while True:
        # Capture frame from the camera
        ret, frame = camera.read() 

        # Convert frame to grayscale for face detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Perform face detection
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5)

        for (x, y, w, h) in faces:
            # Calculate the distance between the face and the stationary box
            distance = max(x + w - (box_x + box_width), box_x - x, y + h - (box_y + box_height), box_y - y)

            if distance > tolerance:
                # If the face is too far from the box, draw a red box and prompt to move closer
                draw_bounding_box(frame, box_x, box_y, box_width, box_height, (0, 0, 255))
                cv2.putText(frame, 'Move closer', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            else:
                # If the face is within the box, draw a green box and prompt to capture
                draw_bounding_box(frame, box_x, box_y, box_width, box_height, (0, 255, 0))
                cv2.putText(frame, 'Face detected! Press "c" to capture', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                # Wait for 'c' key to be pressed to capture face
                if cv2.waitKey(1) & 0xFF == ord('c'):
                    # Create GUI
                    root = Tk()
                    root.title("Face Encoding")

                    label_name = Label(root, text="Enter your name:")
                    label_name.pack()

                    entry_name = Entry(root)
                    entry_name.pack()

                    label_speed = Label(root, text="Enter your preferred speed (1-10 mph):")
                    label_speed.pack()

                    entry_speed = Entry(root)
                    entry_speed.pack()

                    # Define function to handle capturing
                    def on_capture():
                        name = entry_name.get()
                        speed = entry_speed.get()

                        # Extract face embeddings from the captured frame
                        face_embeddings = extract_face_embeddings(frame)  

                        # Insert face data into the database
                        insert_face_data(name, face_embeddings, speed)  

                        # Release camera and close windows
                        camera.release()
                        cv2.destroyAllWindows()
                        root.destroy()  

                    # Create a button to trigger the capturing process
                    capture_button = Button(root, text="Capture", command=on_capture)
                    capture_button.pack()

                    root.mainloop()

                    return  

        # Display the frame with detected face or prompt
        cv2.imshow("Capture", frame)  

        # Check for key press events
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        elif key == 27:  # Press Esc key to exit
            cv2.destroyAllWindows()
            camera.release()
            return  

    # Release camera and close windows
    camera.release()
    cv2.destroyAllWindows()



# Allows for face recognition
def recognize_face():
    # Initialize the camera
    camera = cv2.VideoCapture(0)  # 0 for default camera

    # Load the pre-trained Haar cascade classifier for face detection
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

    # Define parameters for a stationary box
    box_width, box_height = 300, 300
    screen_width, screen_height = 640, 480
    box_x = int((screen_width - box_width) / 2)
    box_y = int((screen_height - box_height) / 2)

    # Connect to the database using credentials
    db_connection = mysql.connector.connect(
        host="localhost",
        user="jacart",
        passwd="jacart2024",
        database="face_recognition_db"
    )

    # Create a Tkinter window for displaying recognition information
    root = Tk()
    root.title("Face Recognition")

    # Create a label for displaying recognition information
    label_info = Label(root, text="", font=("Arial", 14), fg="white", bg="#450084")
    label_info.pack(fill="both", expand=True, padx=20, pady=20)

    recognizing = False  # Flag to indicate if recognition is in progress

    while True:
        ret, frame = camera.read()  # Capture frame from the camera

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Perform face detection
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5)

        for (x, y, w, h) in faces:
            distance = max(x + w - (box_x + box_width), box_x - x, y + h - (box_y + box_height), box_y - y)

            if distance > tolerance:
                # If the face is too far from the box, draw a red box and prompt to move closer
                draw_bounding_box(frame, box_x, box_y, box_width, box_height, (0, 0, 255))
                cv2.putText(frame, 'Move closer', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            else:
                # If the face is within the box, draw a green box and prompt to recognize
                draw_bounding_box(frame, box_x, box_y, box_width, box_height, (0, 255, 0))
                cv2.putText(frame, 'Face detected! Press "r" to recognize', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                if cv2.waitKey(1) & 0xFF == ord('r'):
                    # Perform recognition only once per press
                    if not recognizing:  
                        recognizing = True

                        # Extract face embeddings from the captured frame
                        result = DeepFace.represent(img_path=frame, model_name='Facenet', enforce_detection=True)
                        face_embeddings = result[0]['embedding'] if isinstance(result, list) else result['embedding']

                        cursor = db_connection.cursor()

                        # Query the database to retrieve stored face data
                        query_get_data = "SELECT person_name, preferred_speed, face_encoding FROM face_detect_live"
                        cursor.execute(query_get_data)
                        results = cursor.fetchall()

                        # Set tolerance level for recognizing a face
                        emotion_tolerance = 0.6  

                        for result in results:
                            # Load stored face embeddings from the database
                            db_face_embeddings = pickle.loads(result[2])

                            # Compare face embeddings for recognition
                            similarity_score = 1 - cosine(face_embeddings, db_face_embeddings)

                            if similarity_score > emotion_tolerance:
                                # If a match is found, display recognition information
                                recognized_name = result[0]
                                preferred_speed = result[1]
                                recognition_info = f"Person recognized: {recognized_name}\nPreferred Speed: {preferred_speed}"
                                label_info.config(text=recognition_info)
                                label_info.configure(bg=jmu_gold, fg=jmu_purple)
                                break
                        else:
                            # If no match is found, indicate that the person is not recognized
                            label_info.config(text="Person not recognized")
                            label_info.configure(bg="#450084", fg="white")

                        # Reset recognizing flag for next recognition attempt
                        recognizing = False  

                        # Release camera and close windows
                        camera.release()
                        cv2.destroyAllWindows()
                        # Start Tkinter main loop for GUI
                        root.mainloop()  
                        return

        # Display the frame with face detection or recognition prompts
        cv2.imshow("Recognition", frame)  

        # Check for key press events
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
              # Press Esc key to exit
        elif key == 27:
            cv2.destroyAllWindows()
            camera.release()
            # Destroy Tkinter window
            root.destroy()  
            return 

    # Close database connection, release camera, and close windows
    db_connection.close()
    camera.release()
    cv2.destroyAllWindows()
    root.mainloop() 


# Function to start either encoding or recognition process
def start_process(choice):
    if choice == "Encode":
         # Calls the function to encode faces
        encode_face() 
    elif choice == "Recognize":
        # Calls the function to recognize faces
        recognize_face()  

# Button click handler for encoding or recognition
def handle_click(choice):
     # Calls the start_process function with the selected choice
    start_process(choice) 
# Define color codes for JMU colors
jmu_purple = "#450084"
jmu_gold = "#CBB677"

 # Set a tolerance level for face recognition
tolerance = 50 

# GUI creation function
def create_gui():
    # Create a Tkinter window
    root = Tk()
    root.title("Facial Recognition System")

    # Create a frame with JMU colors
    frame = Label(root, bg=jmu_gold, bd=2, relief="ridge")
    frame.pack(pady=50)

    # Define fonts for labels and buttons
    label_font = ("Arial", 18, "bold")
    button_font = ("Arial", 12)

    # Create a label for the title
    label = Label(frame, text="Facial Recognition System", font=label_font, bg=jmu_gold, fg=jmu_purple)
    label.pack(pady=20)

    # Define common button style
    button_style = {
        "font": button_font,
        "width": 15,
        "height": 2,
        "bg": jmu_gold,
        "fg": jmu_purple,
        "activebackground": "white",
        "borderwidth": 0,
        "highlightthickness": 0,  # Removes border highlight
    }

    # Create buttons for encoding and recognition, with click handlers
    encode_button = Button(frame, text="Encode", command=lambda: handle_click("Encode"), **button_style)
    encode_button.pack(pady=10)

    recognize_button = Button(frame, text="Recognize", command=lambda: handle_click("Recognize"), **button_style)
    recognize_button.pack(pady=10)

    root.mainloop()

# Call the GUI creation function to start the application
create_gui()