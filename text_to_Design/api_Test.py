import google.generativeai as genai
import cv2
import os
from api import api_keys
import PIL.Image
import numpy as np

# Initialize the webcam
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

# Get the API key and configure generative AI
ak = api_keys()
api_key_1 = ak.get_private_vars()

os.environ['GOOGLE_API_KEY'] = api_key_1
genai.configure(api_key=api_key_1)

# Initialize the generative model
model = genai.GenerativeModel(model_name="gemini-1.5-flash")

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture image.")
        break

    # Display the resulting frame
    cv2.imshow('Webcam Feed', frame)

    # Convert frame to an image format that can be processed (PIL Image)
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img = PIL.Image.fromarray(img)
    img_path = "temp_frame.png"
    img.save(img_path)

    # Use the generative model to describe the image
    response = model.generate_content(["if you see a greeen bottle bound a box around it on the frame, and write the coordinates of bottels center",img])

    # Print the generated description
    print("Model says:", response.text)

    # Press 'q' on the keyboard to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
if os.path.exists(img_path):
    os.remove(img_path)
