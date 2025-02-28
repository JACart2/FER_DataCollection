"""OpenAI call point for other functions.

Author: John Rosario Cruz
Version: 2/20/2025
"""
from openai import OpenAI

from io import BytesIO
from PIL import Image
import base64

import sys

## REQUIRES dev acc info
# client = OpenAI()
"""
    client = OpenAI(
        api_key=openai_secret
    )
"""

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

def call(frame):
    # Getting the Base64 string
    base64_image = encode_image(frame)

    ## use this to verify the base64 encoded image works
    # with open('image_base64.txt', 'w') as f:
    #     f.write(base64_image)
    # print("Base64 string written to image_base64.txt")

    # response = client.chat.completions.create(
    #     model="gpt-4o-mini",
    #     messages=[
    #         {
    #             "role": "user",
    #             "content": [
    #                 {
    #                     "type": "text",
    #                     "text": "What is in this image?",
    #                 },
    #                 {
    #                     "type": "image_url",
    #                     "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"},
    #                 },
    #             ],
    #         }
    #     ],
    # )
    
    print("Frame recieved!")
    return "ALERT ALERT ALERT"

if __name__ == '__main__':
    call()