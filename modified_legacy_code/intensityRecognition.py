from deepface import DeepFace

def analyze(path):
    obj = DeepFace.analyze(
    img_path = path, 
    actions = ['emotion'],
    )

    return obj


if __name__ == '__main__':
    path = r"C:\Users\gr8jj\OneDrive\Desktop\SPRING 2025\CS497\fake_faces\angry_face.jpg"
    print(DeepFace.extract_faces(path))
    #print(analyze(r"C:\Users\gr8jj\OneDrive\Desktop\SPRING 2025\CS497\fake_faces\angry_face.jpg"))