import requests
import json

# PascalCase za poimenovanje class-ov.
class comunication:
    def __init__(self) :
        pass

    # camelCase za poimenovanje metod, spremenljivk, ...
    def update_values(self):
        try:
            x = requests.get('http://192.168.1.20:5000/zalij')
            
            if len(x._content.decode())!=0:
                self.data=json.loads(x._content)
                #print(type(self.data))
        except:
            print("will retry in a second")
