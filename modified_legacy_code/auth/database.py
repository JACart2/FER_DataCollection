import mysql.connector 
import os
from dotenv import load_dotenv

load_dotenv()
db_password  = os.environ.get("MY_ENV_VAR") # This is you local .env file database password
db_host = os.environ.get("MY_ENV_HOST") # This is your local .env file database host name
db_user = os.environ.get("MY_ENV_USER") # This is your local .env file database user

# Attempts to connect to the database using the above credentials
def connect_to_database():
    try:
        return mysql.connector.connect(host = db_host, user = db_user, password = db_password)
    except mysql.connector.Error as ConnectionError:
        print("Error: ", ConnectionError)

# Prints the connection to the database if successful or not
print(connect_to_database())
