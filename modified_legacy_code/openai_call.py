"""OpenAI call point for other functions.

Author: John Rosario Cruz
Version: 2/20/2025
"""
from openai import OpenAI

def call():
    ## sample openAI call
    # prob 4o

    """
    client = OpenAI(
        # This is the default and can be omitted
        api_key=openai_secret
    )

    chat_completion = client.chat.completions.create(
        messages=[
            {
                "role": "developer",
                "content": "You are an assistant for a business owner helping derive insight from their online reviews."
                },
            {
                "role": "user",
                "content": prompt,
                }
        ],
        model="gpt-4o-mini",
        max_tokens=2000,
        top_p=0.5,
        frequency_penalty=1,
        presence_penalty=1
    )

    ## https://platform.openai.com/docs/api-reference/chat/object 
    response_string = chat_completion.choices[0].message.content

    return response_string
    """

    return "Demo Photos Woohoo!"

if __name__ == '__main__':
    call()