import tkinter as tk
from tkinter import ttk
import hash

def popupAuthWindow():
    win = tk.Toplevel()
    win.wm_title("Auth Window")
    win.geometry("250x250")
    text_field = tk.Entry(win, width=30, text="Password", textvariable=password_input) #, show="*"
    text_field.pack(pady=10)
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