import requests

response = requests.get("https://api.example.com/users")

print(response.status_code)
print(response.json())      # если JSON
print(response.text)        # если текст
params = {"page": 1, "limit": 10}

response = requests.get(
    "https://api.example.com/users",
    params=params
)

print(response.url)  # увидишь сформированный URL
response = requests.get("https://api.example.com/data")

response.raise_for_status()  # кинет исключение если 4xx/5xx

data = response.json()
requests.get("https://api.example.com", timeout=5)
