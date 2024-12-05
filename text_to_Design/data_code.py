import pandas as pd
import requests
from PIL import Image
from io import BytesIO
import os
import google.generativeai as genai
from tqdm import tqdm
import time
from api import api_keys

# Function to download an image from the URL
def get_image_from_url(url):
    try:
        response = requests.get(url)
        response.raise_for_status()  # Check if the request was successful
        image = Image.open(BytesIO(response.content))
        return image
    except requests.exceptions.RequestException as e:
        print(f"Error fetching the image: {e}")
        return None

# Function to analyze image using Gemini API and extract numerical entity value
def analyze_image(image, query, model):
    try:
        # Sending the image and query to the model to generate a response
        prompt = f"your task is to see inside the image and whatever entity you can see there is an answer written on the image itself, see the image ans answer this question wheter in the image can you see the height, weight etc writtten don't need to answer the height of image in pixels or alll see in the imaeg in whihc unit it is written and write the answer along with the unit too here is the quere for this image, can you see the image and find the entity's {query}"
        response = model.generate_content([image, prompt])
        # Process response to extract only numerical part (Assuming the model returns a string)
        # You can modify this extraction based on the expected response format
        result = response.text
        # Extracting only the numerical value and unit
        numeric_value = ''.join([char for char in result if char.isdigit() or char == '.'])
        return numeric_value
    except Exception as e:
        print(f"Error analyzing image: {e}")
        return None

# Main function to execute the task
def main():
    # Set up the Gemini model API
    ak = api_keys()
    api_key_1 = ak.get_private_vars()
    os.environ['GOOGLE_API_KEY'] = api_key_1
    genai.configure(api_key=api_key_1)
    model = genai.GenerativeModel(model_name="gemini-1.5-flash")

    # Read the input CSV file containing image links, queries, etc.
    df = pd.read_csv(r'text_to_Design/test.csv')

    # Ensure there's an 'analysis_result' column to store results
    if 'analysis_result' not in df.columns:
        df['analysis_result'] = ''

    # Initialize rate limiting variables
    processed_count = 0
    start_time = time.time()

    # Loop through each row of the dataset
    for index, row in tqdm(df.iterrows(), total=len(df), desc="Processing images"):
        image_url = row['image_link']
        query = row['entity_name']

        # Fetch the image from the URL
        image = get_image_from_url(image_url)
        if image:
            # Analyze the image with the provided query
            result = analyze_image(image, query, model)
            # Store the result in the 'analysis_result' column
            df.at[index, 'analysis_result'] = result

        # Save intermediate results after each image processing
        df.to_csv('output2.csv', index=False)

        # Increment processed count and handle rate-limiting
        processed_count += 1

        # Handle 55 prompts per minute rate limiting
        if processed_count >= 55:
            elapsed_time = time.time() - start_time
            if elapsed_time < 60:
                time.sleep(60 - elapsed_time)  # Wait for remaining time in the minute
            processed_count = 0  # Reset count and start time after every 55 requests
            start_time = time.time()
        else:
           
            time.sleep(4)  # 4-second delay to respect the prompt limit

    print("Analysis complete. Results saved to output.csv")

# Run the main function
if __name__ == "__main__":
    main()
