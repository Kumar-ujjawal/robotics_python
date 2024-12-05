import csv
import os
import time
from typing import Dict, Set, Tuple
from tqdm import tqdm
import google.generativeai as genai
from api import api_keys

# Define the entity_unit_map
entity_unit_map: Dict[str, Set[str]] = {
    'width': {'centimetre', 'foot', 'inch', 'metre', 'millimetre', 'yard'},
    'depth': {'centimetre', 'foot', 'inch', 'metre', 'millimetre', 'yard'},
    'height': {'centimetre', 'foot', 'inch', 'metre', 'millimetre', 'yard'},
    'item_weight': {'gram', 'kilogram', 'microgram', 'milligram', 'ounce', 'pound', 'ton'},
    'maximum_weight_recommendation': {'gram', 'kilogram', 'microgram', 'milligram', 'ounce', 'pound', 'ton'},
    'voltage': {'kilovolt', 'millivolt', 'volt'},
    'wattage': {'kilowatt', 'watt'},
    'item_volume': {'centilitre', 'cubic foot', 'cubic inch', 'cup', 'decilitre', 'fluid ounce', 'gallon', 'imperial gallon', 'litre', 'microlitre', 'millilitre', 'pint', 'quart'}
}

allowed_units: Set[str] = {unit for entity in entity_unit_map for unit in entity_unit_map[entity]}

# Initialize Gemini model
ak = api_keys()
api_key_1 = ak.get_private_vars()
os.environ['GOOGLE_API_KEY'] = api_key_1
genai.configure(api_key=api_key_1)
model = genai.GenerativeModel(model_name="gemini-1.5-flash")

def extract_measurement_with_gemini(question: str, context: str) -> Tuple[float, str]:
    """Use Gemini to extract numerical value and unit from text."""
    prompt = f"""
    Task: Extract the measurement (numerical value and unit) from the given context that answers the question.
    Question: {question}
    Context: {context}
    
    Rules:
    1. The answer should be a numerical value followed by a unit.
    2. If multiple measurements are present, choose the one that best answers the question.
    3. If no measurement is found, return "None, None".
    4. In the context you can find the answer to every problem followed by a unit so look for a numerical value followed by a unit and that's your answer 
    Output format: value, unit
    Example output: 500, gram
    
    Your answer:
    """
    
    try:
        response = model.generate_content(prompt)
        answer = response.text.strip()
        
        try:
            value, unit = answer.split(',')
            return float(value.strip()), unit.strip().lower()
        except ValueError:
            return None, None
    except Exception as e:
        print(f"Error in generating content: {str(e)}")
        return None, None

def normalize_unit(unit: str) -> str:
    """Normalize unit to match the allowed units."""
    unit_mapping = {
        'g': 'gram',
        'kg': 'kilogram',
        'mg': 'milligram',
        'ml': 'millilitre',
        'l': 'litre',
        'cm': 'centimetre',
        'mm': 'millimetre',
        'm': 'metre',
        'in': 'inch',
        'oz': 'ounce',
        'lb': 'pound',
        'v': 'volt',
        'kv': 'kilovolt',
        'mv': 'millivolt',
        'w': 'watt',
        'kw': 'kilowatt'
    }
    
    normalized = unit_mapping.get(unit, unit)
    return normalized if normalized in allowed_units else unit

def format_answer(value: float, unit: str, entity: str) -> str:
    """Format the answer according to the entity type."""
    normalized_unit = normalize_unit(unit)
    if normalized_unit in entity_unit_map.get(entity, set()):
        return f"{value} {normalized_unit}"
    else:
        return f"{value} {unit}"

def process_row(row):
    """Process each row and return the result."""
    group_id = row['group_id']
    question = row['questions']
    context = row['context']
    
    entity = question.split()[-1]  # Assuming the entity is the last word in the question
    value, unit = extract_measurement_with_gemini(question, context)
    
    if value is not None and unit is not None:
        answer = format_answer(value, unit, entity)
    else:
        answer = "Unable to extract measurement"
    
    return {
        'group_id': group_id,
        'question': question,
        'answer': answer
    }

def process_csv(input_file: str, output_file: str):
    """Process the input CSV and write results to output CSV with rate limiting."""
    processed_count = 0
    start_time = time.time()

    with open(input_file, 'r', newline='', encoding='utf-8') as infile, \
         open(output_file, 'w', newline='', encoding='utf-8') as outfile:
        
        reader = csv.DictReader(infile)
        fieldnames = ['group_id', 'question', 'answer']
        writer = csv.DictWriter(outfile, fieldnames=fieldnames)
        writer.writeheader()
        
        for row in tqdm(reader, desc="Processing rows"):
            result = process_row(row)
            writer.writerow(result)
            print(result)  # Output to console
            
            processed_count += 1

            # Save after each row
            outfile.flush()

            # Handle 55 prompts per minute rate-limiting
            if processed_count >= 55:
                elapsed_time = time.time() - start_time
                if elapsed_time < 60:
                    time.sleep(60 - elapsed_time)  # Wait for remaining time in the minute
                processed_count = 0  # Reset count and start time after every 55 requests
                start_time = time.time()
            else:
                time.sleep(0.1)  # 1-second delay to respect the prompt limit

# Usage
input_file = r"C:\Users\kumar\OneDrive\Pictures\Screenshots\ROBOTICS\VQA_Dataset.csv"  # Replace with your input file path
output_file = 'output.csv'
process_csv(input_file, output_file)