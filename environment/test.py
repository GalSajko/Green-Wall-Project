import requests
import json


arduino_ip_6 = '192.168.1.14'

arduino_url = f'http://{arduino_ip_6}:5000/'
response = requests.get(arduino_url, timeout=15)
#data = json.loads(response.text)
print(response.content)
