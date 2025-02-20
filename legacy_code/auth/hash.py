import hashlib

# Add auth0 to presentation for next sprint
# Implement mysql password comparison
def hashPassword(password):
    convert_to_bytes = str.encode(password)

    hash_password = hashlib.sha256()
    hash_password.update(convert_to_bytes)
    print(hash_password.digest())
    # test if they are the same password
    # m = hashlib.sha256()
    # m.update(b"this is a test run")
    # print(m.digest())

    return hash_password.digest()