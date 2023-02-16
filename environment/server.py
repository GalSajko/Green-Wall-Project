from flask import Flask, request,jsonify
values =[]
minVal=[]
app = Flask(__name__)
def getMin():
    """
    finds the lowest sensor value from the list of lowest values that were gathered from arduinos since the last call of this function and resets said list.
    Returns:
        list: information about the sensor with the lowest value out of all arduinos
    """
    minVal=[]
    if len(values)!=0:
        min_val = values[0]
        for i in values:
            if i[3]<minVal[3]:
                minVal=i
        values.clear()
    return minVal
def parseData(data,ip):
    """_summary_

    Args:
        data (JSON): data from arduino in JSON format_
        ip (string): arduino address

    Returns:
        list: info about the sensor with the lowest value from a single arduino
    """
    parsed = {}
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
    