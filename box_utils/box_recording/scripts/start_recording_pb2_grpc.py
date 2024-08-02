import grpc
from grpc.framework.common import cardinality
from grpc.framework.interfaces.face import utilities as face_utilities

import start_recording_pb2 as start__recording__pb2
import start_recording_pb2 as start__recording__pb2


class StartRecordingStub(object):

    def __init__(self, channel):
        """Constructor.

        Args:
          channel: A grpc.Channel.
        """
        self.SendMessage = channel.unary_unary(
            "/start_recording.StartRecording/SendMessage",
            request_serializer=start__recording__pb2.StartRecordingRequest.SerializeToString,
            response_deserializer=start__recording__pb2.StartRecordingResponse.FromString,
        )


class StartRecordingServicer(object):

    def SendMessage(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")


def add_StartRecordingServicer_to_server(servicer, server):
    rpc_method_handlers = {
        "SendMessage": grpc.unary_unary_rpc_method_handler(
            servicer.SendMessage,
            request_deserializer=start__recording__pb2.StartRecordingRequest.FromString,
            response_serializer=start__recording__pb2.StartRecordingResponse.SerializeToString,
        ),
    }
    generic_handler = grpc.method_handlers_generic_handler("start_recording.StartRecording", rpc_method_handlers)
    server.add_generic_rpc_handlers((generic_handler,))
