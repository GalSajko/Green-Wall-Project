# Pri nekaterih funkcijah manjka dokumentacija.
from flask import Flask, request, jsonify, render_template
import wall

values = []
minVal = []

# Presledki pred in po =.
visualisationValues=[[[]]]
app = Flask(__name__)
app.secret_key=b'asdasgfascajv'
# Prazna vrstica.
def updateVisualisationValues(data, ip):
    # Ta prazna vrstica je odvec.
    arduino = 0
    
    # Vse IP naslove, ki bodo fiksno doloceni (arduino, server, ...) shrani kot konstante, najbolje v config.py.
    # Pomisli tudi na to, da bos na koncu imel 6 Arduinov. Razmisli, kako bi mappal IP naslov v id arduinota, brez uporabe 6-ih if stavkov.
    if ip == "192.168.1.10":       
        arduino = 1
    elif ip == "192.168.1.11":
        arduino = 2
        # Prazna vrstica (med if-elif in try-except bloki).
    try:
        visualisationValues.pop(arduino)
    # Med try-except ni prazne vrstice - vse je del enega try-except bloka.
    except:
        print("New arduino joined")
    # Prazna vrstica, presledek za vejico.
    visualisationValues.insert(arduino,[])
    # range(0, 5) je enako kot samo range(5) - velja tudi za spodaj.
    for i in range(0, 5):
        visualisationValues[arduino].insert(i,[])
        # Pazi - range(x, y) vkljucuje x, y pa ne (probaj kaj ti sprinta koda for i in range(5, 10): print(i))
        for j in range(53, 60):
            try:
                # Glej komentar spodaj glede t.i. 'magic number-jev'. Naslove senzorjev lahko shranis v array in ga definiras kot konstanto (npr. WALL_SENSORS_IDS = [54, 55, 56, 57]).
                if data["vrstica" + str(i)]["senzor" + str(j)]["id"] == 54:
                    # Presledki za vejico (argumenti insert() funkcije).
                    visualisationValues[arduino][i].insert(0,1)
                elif data["vrstica" + str(i)]["senzor" + str(j)]["id"] == 55:
                    visualisationValues[arduino][i].insert(1,1)
                elif data["vrstica" + str(i)]["senzor" + str(j)]["id"] == 56:
                    visualisationValues[arduino][i].insert(2,1)
                elif data["vrstica" + str(i)]["senzor" + str(j)]["id"] == 57:
                   visualisationValues[arduino][i].insert(3,1) 
            except:
                continue
    # Vem, da si se v fazi razvijanja. Ko bos koncal pa pobrisi vse te zakomentirane vrstice kode.
    #print(visualisationValues)
    return visualisationValues
    #print(dataJson.data)
            
# Ena prazna vrstica med metodami in nasploh ostalimi deli kode je dovolj - velja isto za spodaj.   
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
    # Je ta spremenljivka kje v uporabi?
    parsed = {}
    #dataJson = data
    #print(dataJson)
    y = 0
    x = 0
    # Tukaj sklepas, da nobena vrednost nikoli ne bo vecja od 1347865476. Kaj pa ce slucajno bo? Od Python 3.5 naprej obstaja objekt math.inf (aka infinity), za 
    # katerega velja, da je vecji od kateregakoli stevila.
    minCap = 1347865476
    currCap = 0
    parsed["ip"] = ip
 
    # 1.) Definiraj te stevilke kot konstante. Nasploh se izogibaj uporabi t.i. 'magic number', 'magic string' itd. Vse stevilke, stringe itd.,
    # ki imajo nek pomen (konkretno tukaj gre za stevilo vrstic na panelu in pa najmanjsi in najvecji mozen naslov senzorja) shrani v konstanto. Ce se bo enkrat v prihodnosti slucajno spremenila
    # konfiguracija panela, ali pa se bodo spremenili ID-ji senzorjev, bomo vsi tocno vedeli, kje je potrebno to spremeniti in ne bomo rabili iti skoz celo kodo. Prav tako, bos mogoce moral
    # te vrednosti uporabiti se kje v kodi (oz si jih ze, zgoraj v kodi).
    # 2.) Enako kot zgoraj, na hitro premisli, kako bi lahko poiskal minimum in pripadajoce x, y, IP vrednosti brez uporabe dvojne for zanke.  
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
    # Je return tukaj potreben?
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
    
    # Presledek pred in po =.
    return render_template('index.html', datas=dataJson)

if __name__ == '__main__':
    app.run(host = '192.168.1.20', port = 5000, debug = True)
    

    