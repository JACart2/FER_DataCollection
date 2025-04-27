FROM python:3.10-slim

WORKDIR /app

# Copy everything into the container
COPY . /app

# Copy posture script
COPY posture_with_stop.py /app/

# Upgrade pip
RUN pip install --upgrade pip

# Install the ZED SDK Python binding (pyzed) from the .whl file
RUN pip install pyzed-4.2-cp310-cp310-linux_x86_64.whl

# Install the remaining dependencies from requirements.txt
RUN pip install -r requirements.txt

CMD ["python", "posture_with_stop.py"]

