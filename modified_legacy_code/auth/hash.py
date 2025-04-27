import hashlib
from argon2 import PasswordHasher

# Implement mysql password comparison
def hashPassword(password_input):
    ph = PasswordHasher()
    hash_password = ph.hash(password=password_input)
    print("The password is now hashed as: ", hash_password)
    temp_password = "Test"
    
    try:
        ph.verify(hash_password, temp_password)
        print("They are the same")
    except:
        print("Passwords are different")

    return hash_password