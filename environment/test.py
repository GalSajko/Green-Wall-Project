import requests
import json

arduino_ip_6 = '192.168.1.13'
arduino_ip_1 = '192.168.1.11'
arduino_url = f'http://{arduino_ip_6}:5000/'
response = requests.get(arduino_url)
#data = json.loads(response.text)
print(response.content)
