import json
import requests

headers = {"Authorization": "Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJ1c2VyX2lkIjoiNzUxYWExZWMtYTZhZi00M2FlLTllYzUtZDAzYjQzMTNjNjgwIiwidHlwZSI6ImFwaV90b2tlbiJ9.j0zYLNpc9VsfHqowLraEpQ_OD1Mbo07MXoUW-jjEAZ0"}

url = "https://api.edenai.run/v2/image/question_answer"
json_payload = {
    "providers": "alephalpha",
    "file_url": "https://m.media-amazon.com/images/I/612mrlqiI4L.jpg",
    "question": "What is the visble item_weight in the image ?",
}

response = requests.post(url, json=json_payload, headers=headers)

result = json.loads(response.text)
print(result['alephalpha']['answers'])
