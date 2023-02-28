from flask import Flask, request, jsonify, render_template
import math
import random
from flask_socketio import SocketIO
import requests
import time
import json
import sys
import threading
import arduinoVal
sys.path.append('..')
import config
import numpy as np
config.minValues = []
minVal = []
config.visualisationValues= np.zeros((7,6,6), dtype = int).tolist()
config.arduinoValues = [{},{},{},{},{},{},{}]
config.arduinoTimes = np.zeros((7), dtype = int).tolist()
sensorIDs = [54, 55, 56, 57, 58, 59]
arduinoNum = 6
arduinoVal.sensors = []
arduinoVal.last = 0

def getDataFromArduino():
    """Function sends GET requests to arduinos and saves timestamps and sensor values when replies arrive. If the arduino doesnt send a reply for 70 seconds it is treated as not working.
    """
    while True:
        for i in config.ARDUIONO_IP_LIST:
            dataNew = ""
            url = f'http://{i}:5000/'
            try:
                response = requests.get(url, timeout = 30)
                print(response.content)
                time.sleep(5)
            except:
                config.arduinoValues[int(i[len(i)-1])] = {}
                
            

th = threading.Thread(target = getDataFromArduino)
th.start()

app = Flask(__name__)
app.secret_key = b'asdasgfascajv'

def updateVisualisationValues(arduinoInd, arduinoData):
    """Function inserts value 1 to for every sensor currently connected to the arduino.

    Args:
        data (JSON): Data from arduino in JSON format.
        ip (String): Arduino address.

    Returns:
        list: List with sensor locations.
    """  
    for i in range(arduinoNum):
        for j in sensorIDs:
            try:
                arduinoData["vrstica" + str(i)]["senzor" + str(j)]["id"]
                config.visualisationValues[arduinoInd][i][sensorIDs.index(j)] = 1
                arduinoVal.sensors.append([arduinoInd, i, j])
            except:
                config.visualisationValues[arduinoInd][i][sensorIDs.index(j)] = 0
    return config.visualisationValues

def getMin():
    """
    Finds the lowest sensor value from the list of lowest values that were gathered from arduinos since the last call of this function and resets said list.
    Returns:
        list: Information about the sensor with the lowest value out of all arduinos.
    """
    minVal = []
    if len(config.minValues) != 0:
        minVal = config.minValues[0]
        for i in config.minValues:
            if i[3] < minVal[3]:
                minVal = i
        config.minValues.clear()
    return minVal

def parseData(data, ind):
    """
    Finds the lowest value from selected arduino.

    Args:
        data (JSON): Data from arduino in JSON format.
        ip (string): Arduino address.

    Returns:
        list: Information about the sensor with the lowest value from a single arduino.
    """
    y = 0
    x = 0
    minCap = math.inf
    currCap = 0
    for i in range(arduinoNum):
        for j in sensorIDs:
            try:
                currCap = data["vrstica" + str(i)]["senzor" + str(j)]["cap"]
            except:
                currCap = 0
            if currCap < minCap and currCap != 0:
                minCap = currCap
                y = i
                x = data["vrstica" + str(i)]["senzor" + str(j)]["id"]
    config.minValues.append([ind, y, x, minCap])

@app.route('/spiderPos', methods=['POST', 'GET'])
def getSpiderPos():
    """Gets data from the spider and forwards it to the frontend for visualisation.

    Returns:
        JSON: Position of the spider.
    """
    pins=[]
    if request.method == 'POST':
        pins = json.loads(request.get_data().decode())
        config.poseData = pins["pose"]
        return 'OK'
    elif request.method == 'GET':
        try:
            return jsonify(config.poseData), 200, {"Access-Control-Allow-Origin": "*"}
        except:
            return jsonify([]), 200, {"Access-Control-Allow-Origin": "*"}

@app.route('/zalij',methods=["GET"])
def getVal():
    """Sends the location of the sensor with the smallest value.

    Returns:
       list: Information about the sensor with the lowest value out of all arduinos.
    """
    for i in range(len(config.arduinoValues)):
        parseData(config.arduinoValues[i],i)
    try:
        return jsonify(getMin()), 200, {"Access-Control-Allow-Origin": "*"}
    except:
        return jsonify([]), 200, {"Access-Control-Allow-Origin": "*"}
    
@app.route('/update')
def update():
    """Sends locations of active sensors to frontend.

    Returns:
        JSON: Information about active sensors.
    """  
    arduinoVal.sensors.clear()
    for i in range(len(config.arduinoValues)):
        if config.arduinoValues[i] != 0:
            config.dataJson = updateVisualisationValues(i, config.arduinoValues[i])
    try:
        return jsonify(config.dataJson), 200, {"Access-Control-Allow-Origin": "*"}
    except:
        return jsonify([]), 200, {"Access-Control-Allow-Origin": "*"}

@app.route('/ping',methods=["GET"])
def pingPong():
    """Route for testing connection.

    Returns:
        JSON: Sends the string 'pong!'
    """
    return jsonify('pong!'), 200, {"Access-Control-Allow-Origin": "*"}

@app.route('/index')
def index():
    """Route for serving the frontend page.

    Returns:
        render_template: serves a webpage
    """
    

    return render_template('index.html')

@app.route('/test', methods=['POST'])
def handle_data():
    """receives data and calls the parse function
    Returns:
        string: reply to arduino after getting the data
    """
    ip = request.remote_addr
    index = int(ip[len(ip)-1])
    data = request.get_json()
    try:
        config.arduinoValues[index].update(data) 
        config.arduinoTimes[int(ip[len(ip)-1])] = time.time()
    except:
        config.arduinoValues[index] = {}
    return "OK" 

if __name__ == '__main__':
    app.run(host = '192.168.1.20', port = 5000, debug = True)
    

    