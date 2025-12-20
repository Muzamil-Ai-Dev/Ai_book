# Use python 3.11 slim image
FROM python:3.11-slim

# Set the working directory to /code
WORKDIR /code

# Copy the requirements file into the container at /code
COPY apps/api/requirements.txt /code/requirements.txt

# Install the dependencies
RUN pip install --no-cache-dir --upgrade -r /code/requirements.txt

# Copy the apps/api directory into the container at /code/apps/api
COPY apps/api /code/apps/api

# Set the working directory to the api app location
WORKDIR /code/apps/api

# Expose port 7860 for Hugging Face Spaces
EXPOSE 7860

# Run the application
CMD ["uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "7860"]
