import grpc
from grpc.framework.common import cardinality
from grpc.framework.interfaces.face import utilities as face_utilities

import recording_status_pb2 as recording__status__pb2
import recording_status_pb2 as recording__status__pb2


class RecordingStatusStub(object):

    def __init__(self, channel):
        """Constructor.

        Args:
          channel: A grpc.Channel.
        """
        self.SendMessage = channel.unary_unary(
            "/recording_status.RecordingStatus/SendMessage",
            request_serializer=recording__status__pb2.RecordingStatusRequest.SerializeToString,
            response_deserializer=recording__status__pb2.RecordingStatusResponse.FromString,
        )


class RecordingStatusServicer(object):

    def SendMessage(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")


def add_RecordingStatusServicer_to_server(servicer, server):
    rpc_method_handlers = {
        "SendMessage": grpc.unary_unary_rpc_method_handler(
            servicer.SendMessage,
            request_deserializer=recording__status__pb2.RecordingStatusRequest.FromString,
            response_serializer=recording__status__pb2.RecordingStatusResponse.SerializeToString,
        ),
    }
    generic_handler = grpc.method_handlers_generic_handler("recording_status.RecordingStatus", rpc_method_handlers)
    server.add_generic_rpc_handlers((generic_handler,))
