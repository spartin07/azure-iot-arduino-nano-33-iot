# import os
import json
from azure.servicebus import ServiceBusClient, ServiceBusMessage

CONNECTION_STR = "Endpoint=sb://fuschiaband.servicebus.windows.net/;SharedAccessKeyName=iothubroutes_fuschiaband;SharedAccessKey=mLF+WqZvkMxNXaVTqHmPrchyRGQNjvUWAEPpIVYmFds=;EntityPath=dataqueue"
QUEUE_NAME = "dataqueue"

#Sends a single message to the serviceBusQueue
def send_single_message(sender):
    message = ServiceBusMessage("{\"userId\":9,\"workoutDate\":\"2022-02-24\",\"exercise\":\"sending from python\",\"data\":\"80.81420180.24949679.94495481.54075681.937263135.547379133.213654126.571320124.666946123.524231122.343903121.247246120.231239119.237625118.262039117.183983116.126862\"}")
    sender.send_messages(message)
    print("Sent a single message")

def send_a_list_of_messages(sender):
    messages = [ServiceBusMessage("Message in list") for _ in range(5)]
    sender.send_messages(messages)
    print("Sent a list of 5 messages")

def send_batch_message(sender):
    batch_message = sender.create_message_batch()
    for _ in range(10):
        try:
            batch_message.add_message(ServiceBusMessage("Message inside a ServiceBusMessageBatch"))
        except ValueError:
            # ServiceBusMessageBatch object reaches max_size.
            # New ServiceBusMessageBatch object can be created here to send more data.
            break
    sender.send_messages(batch_message)
    print("Sent a batch of 10 messages")


def handle_recieved_message(msg):


servicebus_client = ServiceBusClient.from_connection_string(conn_str=CONNECTION_STR, logging_enable=True)

with servicebus_client:
    sender = servicebus_client.get_queue_sender(queue_name=QUEUE_NAME)
    with sender:
        send_single_message(sender)
#         send_a_list_of_messages(sender)
#         send_batch_message(sender)
#
# print("Done sending messages")
# print("-----------------------")

with servicebus_client:
    receiver = servicebus_client.get_queue_receiver(queue_name=QUEUE_NAME, max_wait_time=5)
    with receiver:
        for msg in receiver:
            print("Received: " + str(msg))
            d = json.loads(str(msg))
            print(d["userId"], d["data"])
            receiver.complete_message(msg)


"""
Idea here is to read through the serviceBusQueue. Messages with the exercise as 'Start' will signify the start of new exercises.
For now im assuming the message queue will be used only be one device at a time. These means we can combined all messages together
until we see a new start message.

Best way to do so is when started a json file/object is created from the first message. Data from each subsequent message should be
appended to that object.
Either at the end of all the messages or during the whole thing, this should be written to a file. The file title can be created from
userId + exercise + date. Perhaps the contents should just be the data? As i believe thats all that is required for the ML 
"""
