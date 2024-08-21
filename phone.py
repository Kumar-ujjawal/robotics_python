import requests
import phonenumbers
from phonenumbers import geocoder, carrier, timezone

# Replace with the phone number you want to analyze
phone_number = "+919199962215"  # Example number, change this to the actual number

try:
    # Parse the phone number
    parsed_number = phonenumbers.parse(phone_number)
    
    # Get the location of the phone number
    location = geocoder.description_for_number(parsed_number, "en")
    
    # Get the carrier of the phone number
    carrier_name = carrier.name_for_number(parsed_number, "en")
    
    # Get the time zone of the phone number
    time_zones = timezone.time_zones_for_number(parsed_number)
    
    # Print the information
    print(f"Phone Number: {phone_number}")
    print(f"Location: {location}")
    print(f"Carrier: {carrier_name}")
    print(f"Time Zones: {', '.join(time_zones)}")
    
    # Get the user's IP address (as a placeholder for a more specific location source)
    ip_response = requests.get('https://api.ipify.org?format=json')
    ip_data = ip_response.json()
    ip_address = ip_data['ip']
    
    # Get geolocation data based on IP address using ipinfo.io
    response = requests.get(f"https://ipinfo.io/{ip_address}/json")
    
    # Check if the request was successful
    if response.status_code == 200:
        data = response.json()
        print(f"IP Address: {ip_address}")
        print(f"City: {data.get('city')}")
        print(f"Region: {data.get('region')}")
        print(f"Country: {data.get('country')}")
        print(f"Location: {data.get('loc')}")
    else:
        print(f"Failed to get geolocation data. Status code: {response.status_code}")

except phonenumbers.phonenumberutil.NumberParseException as e:
    print(f"Error parsing phone number: {e}")
except Exception as e:
    print(f"An error occurred: {e}")
