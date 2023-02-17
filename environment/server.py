from flask import Flask, request, jsonify, render_template
import wall

values = []
minVal = []

visualisationValues=[[[]]]
app = Flask(__name__)
app.secret_key=b'asdasgfascajv'
def updateVisualisationValues(data, ip):
    
    arduino = 0
    
    if ip == "192.168.1.10":       
        arduino = 1
    elif ip == "192.168.1.11":
        arduino = 2
    try:
        visualisationValues.pop(arduino)
        
    except:
        print("New arduino joined")
    visualisationValues.insert(arduino,[])
    for i in range(0, 5):
        visualisationValues[arduino].insert(i,[])
        for j in range(53, 60):
            try:
                if data["vrstica" + str(i)]["senzor" + str(j)]["id"] == 54:
                    visualisationValues[arduino][i].insert(0,1)
                elif data["vrstica" + str(i)]["senzor" + str(j)]["id"] == 55:
                    visualisationValues[arduino][i].insert(1,1)
                elif data["vrstica" + str(i)]["senzor" + str(j)]["id"] == 56:
                    visualisationValues[arduino][i].insert(2,1)
                elif data["vrstica" + str(i)]["senzor" + str(j)]["id"] == 57:
                   visualisationValues[arduino][i].insert(3,1) 
            except:
                continue
    #print(visualisationValues)
    return visualisationValues
    #print(dataJson.data)
            
    
def getMin():
    """
    Finds the lowest sensor value from the list of lowest values that were gathered from arduinos since the last call of this function and resets said list.
    Returns:
        list: Information about the sensor with the lowest value out of all arduinos.
    """
    minVal = []
    if len(values) != 0:
        minVal = values[0]
        for i in values:
            if i[3] < minVal[3]:
                minVal = i
        values.clear()
    return minVal

def parseData(data, ip):
    """
    Finds the lowest value from selected arduino.

    Args:
        data (JSON): Data from arduino in JSON format.
        ip (string): Arduino address.

    Returns:
        list: Information about the sensor with the lowest value from a single arduino.
    """
    parsed = {}
    #dataJson = data
    #print(dataJson)
    y = 0
    x = 0
    minCap = 1347865476
    currCap = 0
    parsed["ip"] = ip
    for i in range(0, 5):
        for j in range(53, 60):
            try:
                currCap = data["vrstica" + str(i)]["senzor" + str(j)]["cap"]
            except:
                currCap = 0
            if currCap < minCap and currCap != 0:
                minCap = currCap
                y = i
                x = data["vrstica" + str(i)]["senzor" + str(j)]["id"]
    values.append([ip, y, x, minCap])
    return [ip, y, x, minCap]

@app.route('/zalij',methods=["GET"])
def getVal():
    return jsonify(getMin()), 200, {"Access-Control-Allow-Origin": "*"}

@app.route('/ping',methods=["GET"])
def pingPong():
    return jsonify('pong!'), 200, {"Access-Control-Allow-Origin": "*"}




@app.route('/data', methods=['POST', 'GET'])
def handleData():
    """Function receives data and forwards data and the ip to the updateVisualisatonValues and parseData functions.

    Returns:
        string: Reply to arduino after getting the data.
    """
    ip = request.remote_addr
    data = request.get_json()
    
    dataJson = updateVisualisationValues(data, ip)
    
    
    #print(data)
    parseData(data, ip)
    
    return render_template('index.html', datas=dataJson)

if __name__ == '__main__':
    app.run(host = '192.168.1.20', port = 5000, debug = True)
    

    