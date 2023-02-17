from flask import Flask, request,jsonify
from environment import wall
values =[]
minVal=[]
app = Flask(__name__)
# Za cel file - dokumentacijo pisi, kot normalno poved (z veliko zacetnico in piko). Med posameznimi deli kode (metode, importi, ...) naj bodo prazne vrstice.
def getMin():
    """
    finds the lowest sensor value from the list of lowest values that were gathered from arduinos since the last call of this function and resets said list.
    Returns:
        list: information about the sensor with the lowest value out of all arduinos
    """
    minVal=[]
    if len(values)!=0:
        minVal = values[0]
        for i in values:
            if i[3]<minVal[3]:
                minVal=i
        values.clear()
    return minVal
def parseData(data,ip):
    """
    finds the lowest value from selected arduino

    Args:
        data (JSON): data from arduino in JSON format_
        ip (string): arduino address

    Returns:
        list: info about the sensor with the lowest value from a single arduino
    """
    parsed = {}
    # Presledki med in po operatorji.
    y=0
    x=0
    minCap=1347865476
    currCap=0
    parsed["ip"]=ip
    for i in range(0,5):
        for j in range(53,60):
            try:
                currCap=data["vrstica"+str(i)]["senzor"+str(j)]["cap"]
            except:
                currCap=0
            if currCap<minCap and currCap!=0:
                minCap=currCap
                y=i
                x = data["vrstica"+str(i)]["senzor"+str(j)]["id"]
    # Presledki za vejicami.
    values.append([ip,y,x,minCap])
    return [ip,y,x,minCap]
@app.route('/zalij',methods=["GET"])
def getVal():
    return jsonify(getMin()),200,{"Access-Control-Allow-Origin": "*"}
@app.route('/ping',methods=["GET"])
def pingPong():
    return jsonify('pong!'),200,{"Access-Control-Allow-Origin": "*"}
@app.route('/', methods=['POST'])
def handleRequest():
    """default route no special function
    Returns:
        string: reply after getting the data
    """
    data = request.args.get('data')
    print('Received data:', data)
    response = 'Hello, Arduino!'
    return response
@app.route('/data', methods=['POST'])
def handleData():
    """receives data and calls the parse function

    Returns:
        string: reply to arduino after getting the data
    """
    ip = request.remote_addr
    data = request.get_json()
    parseData(data, ip)
    return "OK" 
if __name__ == '__main__':
    app.run(host='192.168.1.20', port=5000, debug=True)
    