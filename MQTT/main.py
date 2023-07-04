import paho.mqtt.client as paho

broker = "broker.hivemq.com"
port = 1883
pesan = ""

def on_message(client, userdata, message):
    msg = str(message.payload.decode("utf-8"))
    t = str(message.topic)
    if (t == "drone"):
        global pesan
        pesan = str(msg)

client= paho.Client("GUI")
client.on_message=on_message

print("connecting to broker ",broker)
client.connect(broker,port)#connect
print(broker," connected")

client.loop_start()
print("Subscribing to topic")
client.subscribe("drone")

while True:
    print(pesan)