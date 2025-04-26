import tkinter as tk
from tkinter import ttk
import hash

from pyzed import sl
import numpy as np
import cv2

# Authentication window for users to enter in password.
# It will check if the password entered matches to what's stored in the database
def popupAuthWindow():
    global win 
    win = tk.Toplevel()
    global password_input 
    password_input = tk.StringVar()


def popupAuthWindow():
    win = tk.Toplevel()

    win.wm_title("Auth Window")
    win.geometry("250x250")
    text_field = tk.Entry(win, width=30, text="Password", textvariable=password_input) #, show="*"
    text_field.pack(pady=10)

  
    submit_button = tk.Button(win, text="Submit", command=lambda: submit(password_input.get()))
    submit_button.pack()

    # Waits for the window to be closed or destroyed by the user
    win.wait_window(win)

    return password_auth

# Method to check user input
def submit(password_input):
    user_input = password_input
    hashcode = hash.hashPassword(user_input)
    global password_auth
    print("The hashcoded password is: " + str(hashcode))
    if hashcode:
        password_auth = True
        win.destroy()
    else:
        print(str(ValueError) + " Passwords are different")
        password_auth = False

    submit_button = tk.Button(win, text = "Submit", command=submit)
    submit_button.pack()

def submit():
    user_input = password_input.get()
    hashcode = hash.hashPassword(user_input)
    print("The hashcoded password is: " + str(hashcode))

root = tk.Tk()
password_input = tk.StringVar()
root.geometry("500x500")
label = tk.Label(root, text="Hello, Tkinter!")
label.pack()


authenticate_button = tk.Button(root, text="Scan Face", command=popupAuthWindow)
authenticate_button.pack()

# scan_face_button = tk.Button(root, text="Scan Face", command=popupAuthWindow)
# scan_face_button.pack()

# help_button = tk.Button(root, text="help", command=test)
# help_button.pack()

root.mainloop()

