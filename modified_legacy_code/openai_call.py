"""OpenAI call point for other functions.

Author: John Rosario Cruz
Version: 3/3/2025
"""
from openai import OpenAI
from io import BytesIO
from PIL import Image
import base64


## REQUIRES dev acc info
client = OpenAI(
    api_key="secret_key"
)


def encode_image(image_tensor):
    """Take the image tensor (taken from the ZED camera), and base64 encode it so it can be passed to the OpenAI API.

    Args:
        image_tensor (np matrix): The image matrix.

    Returns:
        str: the encoded image

    """
    # Convert the numpy array (image tensor) to a PIL Image
    image = Image.fromarray(image_tensor)

    # Create a BytesIO object to save the image as bytes
    buffered = BytesIO()
    image.save(buffered, format="PNG")  # You can change the format if needed (e.g., JPEG)

    # Get the byte data from the buffer
    img_byte_array = buffered.getvalue()

    # Encode image to base64
    img_base64 = base64.b64encode(img_byte_array).decode('utf-8')

    return img_base64

def call_openai(frame):
    """Calls ChatGPT to make a determination on the image of the person.
    
    Args:
        frame (np.matrix): The image represented by a matrix
    
    Returns:
        str: A response from chatGPT (U, I, N, or F)
            >>> U = Unconscious
            >>> I = Incapacitated
            >>> N = Neither
            >>> F = Failure to identify
    
    """
    # Getting the Base64 string
    base64_image = encode_image(frame)

    prompt = f"""
    You are a camera positioned at the front of a golf cart looking inwards at the passenger. 
    You are being passed a base64 encoded version of the image that you must decode to analyze.
    Is the passenger unconscious, incapacitated, or neither. 
    You must make a determination by analyzing the image. 
    Return either U, I, or N (unconscious, incapacitated, or neither).
    If you are incapable of making a determination, return F.
    Return only a single letter (U, I, N, or F) as a response, dont include extra text.
    """

    # ## use this to verify the base64 encoded image works
    # with open('image_base64.txt', 'w') as f:
    #     f.write(base64_image)
    # print("Base64 string written to image_base64.txt")

    response = client.chat.completions.create(
        model="chatgpt-4o-latest",
        messages=[
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": prompt
                    },
                    {
                        "type": "image_url",
                        "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"},
                    },
                ],
            }
        ],
    )

    
    return response.choices[0]

if __name__ == '__main__':
    pass
