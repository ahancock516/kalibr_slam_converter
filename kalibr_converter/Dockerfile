FROM python:3.9-slim

# Set the working directory inside the container.
WORKDIR /scripts

# Copy and install Python dependencies using pip.
COPY requirements.txt .

RUN pip install --no-cache-dir -r requirements.txt

# Set the default command to execute when the container starts.
ENTRYPOINT ["/bin/bash"]