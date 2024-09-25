from concurrent import futures
import grpc
import start_recording_pb2
import start_recording_pb2_grpc


class StartRecordingServicer(start_recording_pb2_grpc.StartRecordingServicer):
    def SendMessage(self, request, context):
        print(f"Received message: {request.message}")
        return start_recording_pb2.StartRecordingResponse(response="Message received")


def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    start_recording_pb2_grpc.add_StartRecordingServicer_to_server(StartRecordingServicer(), server)
    server.add_insecure_port("[::]:50051")
    server.start()
    print("Server started, listening on port 50051.")
    server.wait_for_termination()


if __name__ == "__main__":
    serve()
