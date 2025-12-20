# Use python 3.11 slim image
FROM python:3.11-slim

# Set up a new user named "user" with user ID 1000
RUN useradd -m -u 1000 user

# Switch to the "user" user
USER user

# Set home environment variable
ENV HOME=/home/user \
    PATH=/home/user/.local/bin:$PATH

# Set the working directory to /home/user/app
WORKDIR $HOME/app

# Copy the requirements file into the container
COPY --chown=user apps/api/requirements.txt $HOME/app/requirements.txt

# Install the dependencies
RUN pip install --no-cache-dir --user --upgrade -r $HOME/app/requirements.txt

# Copy the apps/api directory into the container
COPY --chown=user apps/api $HOME/app/apps/api

# Set the working directory to the api app location
WORKDIR $HOME/app/apps/api

# Expose port 7860 for Hugging Face Spaces
EXPOSE 7860

# Run the application
CMD ["uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "7860"]