import mysql.connector 
import os
from dotenv import load_dotenv
load_dotenv()

password  = os.environ.get("MY_ENV_VAR")

conn = mysql.connector.connect (
    host="localhost",
    user="root",
    passwd = password 
)



print(conn)

# conn.close()