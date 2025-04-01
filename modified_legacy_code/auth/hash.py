import hashlib
from argon2 import PasswordHasher
import database as db

# Implements mysql password comparison
def hashPassword(password_input):
    # connects to the database
    connection = db.connect_to_database()
    
    # Prints connection
    print(connection)

    curs = connection.cursor()
    
    # Select column highlighting hascoded password to be compared to user input
    curs.execute("SELECT admin_password FROM users.admin_password LIMIT 1")
    row = curs.fetchone()
    print(row[0])

    ph = PasswordHasher()

    # Verifies the users inputed password and returns its respective boolean value
    try:
        ph.verify(row[0], password_input)
        print("Correct Password")
        bool_statement = True
    except:
        print("Passwords is Incorrect. Please try again")
        bool_statement = False

    # Close the database connection
    curs.close()
    connection.close()
    return bool_statement