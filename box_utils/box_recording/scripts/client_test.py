import grpc
import start_recording_pb2
import start_recording_pb2_grpc


def run():
    with grpc.insecure_channel("localhost:50051") as channel:
        stub = start_recording_pb2_grpc.StartRecordingStub(channel)
        response = stub.SendMessage(
            start_recording_pb2.StartRecordingRequest(topics="test----/tf test----/tf_static", timestamp="World")
        )
        print(f"Server response: {response.response}")


if __name__ == "__main__":
    run()
