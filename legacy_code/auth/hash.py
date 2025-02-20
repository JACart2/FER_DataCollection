import hashlib

m = hashlib.sha256()
m.update(b"this is a test run")
print(m.digest())
