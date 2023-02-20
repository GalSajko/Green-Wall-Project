# Pri nekaterih funkcijah manjka dokumentacija.
from flask import Flask, request, jsonify, render_template
import math
import wall

values = []
minVal = []
visualisationValues = [[[]]]

app = Flask(__name__)
app.secret_key = b'asdasgfascajv'

def updateVisualisationValues(data, ip):
    """Function inserts value 1 to for every sensor currently connected to the arduino.

    Args:
        data (JSON): Data from arduino in JSON format.
        ip (String): Arduino address.

    Returns:
        list: List with sensor locations.
    """
    arduino = int(ip[len(ip)-1])
    try:
        visualisationValues.pop(arduino)
    except:
        print("New arduino joined")
    visualisationValues.insert(arduino,[])
    for i in range(6):
        try:
            visualisationValues[arduino].insert(i, [])
        except IndexError:
            continue
        for j in range(53, 60):
            try:
                # Glej komentar spodaj glede t.i. 'magic number-jev'. Naslove senzorjev lahko shranis v array in ga definiras kot konstanto (npr. WALL_SENSORS_IDS = [54, 55, 56, 57]).
                if data["vrstica" + str(i)]["senzor" + str(j)]["id"] == 54:
                    visualisationValues[arduino][i].insert(0, 1)
                elif data["vrstica" + str(i)]["senzor" + str(j)]["id"] == 55:
                    visualisationValues[arduino][i].insert(1,1)
                elif data["vrstica" + str(i)]["senzor" + str(j)]["id"] == 56:
                    visualisationValues[arduino][i].insert(2,1)
                elif data["vrstica" + str(i)]["senzor" + str(j)]["id"] == 57:
                   visualisationValues[arduino][i].insert(3, 1) 
            except:
                continue
    
    print(visualisationValues)
    return visualisationValues
    #print(dataJson.data)
            
def getMin():
    """
    Finds the lowest sensor value from the list of lowest values that were gathered from arduinos since the last call of this function and resets said list.
    Returns:
        list: Information about the sensor with the lowest value out of all arduinos.
    """
    # Ce prav razumem je values 2d matrika, ti pa isces minimalno vrednost po 3. stolpcu matrike (kapacitivnost). Z uporabo numpy knjiznice lahko to naredis v 1 vrstici, ni pa treba za to izgubljati prevec casa.
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
    y = 0
    x = 0
    minCap = math.inf
    currCap = 0
    # 1.) Definiraj te stevilke kot konstante. Nasploh se izogibaj uporabi t.i. 'magic number', 'magic string' itd. Vse stevilke, stringe itd.,
    # ki imajo nek pomen (konkretno tukaj gre za stevilo vrstic na panelu in pa najmanjsi in najvecji mozen naslov senzorja) shrani v konstanto. Ce se bo enkrat v prihodnosti slucajno spremenila
    # konfiguracija panela, ali pa se bodo spremenili ID-ji senzorjev, bomo vsi tocno vedeli, kje je potrebno to spremeniti in ne bomo rabili iti skoz celo kodo. Prav tako, bos mogoce moral
    # te vrednosti uporabiti se kje v kodi (oz si jih ze, zgoraj v kodi).
    # 2.) Enako kot zgoraj, na hitro premisli, kako bi lahko poiskal minimum in pripadajoce x, y, IP vrednosti brez uporabe dvojne for zanke.  
    for i in range(6):
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
    parseData(data, ip)
    return render_template('index.html', datas = dataJson)

if __name__ == '__main__':
    app.run(host = '192.168.1.20', port = 5000, debug = True)
    

    