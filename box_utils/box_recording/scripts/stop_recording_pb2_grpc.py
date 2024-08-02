import grpc
from grpc.framework.common import cardinality
from grpc.framework.interfaces.face import utilities as face_utilities

import stop_recording_pb2 as stop__recording__pb2
import stop_recording_pb2 as stop__recording__pb2


class StopRecordingStub(object):

    def __init__(self, channel):
        """Constructor.

        Args:
          channel: A grpc.Channel.
        """
        self.SendMessage = channel.unary_unary(
            "/stop_recording.StopRecording/SendMessage",
            request_serializer=stop__recording__pb2.StopRecordingRequest.SerializeToString,
            response_deserializer=stop__recording__pb2.StopRecordingResponse.FromString,
        )


class StopRecordingServicer(object):

    def SendMessage(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")


def add_StopRecordingServicer_to_server(servicer, server):
    rpc_method_handlers = {
        "SendMessage": grpc.unary_unary_rpc_method_handler(
            servicer.SendMessage,
            request_deserializer=stop__recording__pb2.StopRecordingRequest.FromString,
            response_serializer=stop__recording__pb2.StopRecordingResponse.SerializeToString,
        ),
    }
    generic_handler = grpc.method_handlers_generic_handler("stop_recording.StopRecording", rpc_method_handlers)
    server.add_generic_rpc_handlers((generic_handler,))
