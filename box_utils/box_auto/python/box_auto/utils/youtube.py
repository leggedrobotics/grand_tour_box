try:
    import os
    import random
    import sys
    import time
    import pathlib
    import http.client

    import httplib2
    from googleapiclient.discovery import build
    from googleapiclient.errors import HttpError
    from googleapiclient.http import MediaFileUpload
    from oauth2client.client import flow_from_clientsecrets
    from oauth2client.file import Storage
    from oauth2client.tools import run_flow
    from box_auto.utils import BOX_AUTO_DIR

    # Constants
    httplib2.RETRIES = 1
    MAX_RETRIES = 10

    RETRIABLE_EXCEPTIONS = (
        httplib2.HttpLib2Error,
        IOError,
        http.client.NotConnected,
        http.client.IncompleteRead,
        http.client.ImproperConnectionState,
        http.client.CannotSendRequest,
        http.client.CannotSendHeader,
        http.client.ResponseNotReady,
        http.client.BadStatusLine,
    )

    RETRIABLE_STATUS_CODES = [500, 502, 503, 504]

    # AIzaSyD920NjrL7E4aCqEkVKNfLXV_KjMaUTpFM

    CLIENT_SECRETS_FILE = pathlib.Path(BOX_AUTO_DIR) / "../.." / ".secrets/client_secret_leggedrobotics_youtube.json"
    CLIENT_SECRETS_FILE = CLIENT_SECRETS_FILE.resolve()

    YOUTUBE_UPLOAD_SCOPE = "https://www.googleapis.com/auth/youtube.upload"
    YOUTUBE_API_SERVICE_NAME = "youtube"
    YOUTUBE_API_VERSION = "v3"

    MISSING_CLIENT_SECRETS_MESSAGE = f"""
    WARNING: Please configure OAuth 2.0

    To make this sample run you will need to populate the client_secrets.json file
    found at:

    {CLIENT_SECRETS_FILE}

    with information from the API Console:
    https://console.cloud.google.com/

    For more information, see:
    https://developers.google.com/api-client-library/python/guide/aaa_client_secrets
    """

    VALID_PRIVACY_STATUSES = ("public", "private", "unlisted")
except:
    pass

DESCRIPTION = """More Information about the GrandTour: https://grand-tour.leggedrobotics.com/\r\nVideo uploaded by Jonas Frey, ETH Zurich."""


def get_authenticated_service():
    flow = flow_from_clientsecrets(
        str(CLIENT_SECRETS_FILE), scope=YOUTUBE_UPLOAD_SCOPE, message=MISSING_CLIENT_SECRETS_MESSAGE
    )

    storage = Storage(f"{sys.argv[0]}-oauth2.json")
    credentials = storage.get()

    if credentials is None or credentials.invalid:
        credentials = run_flow(flow, storage)

    return build(YOUTUBE_API_SERVICE_NAME, YOUTUBE_API_VERSION, http=credentials.authorize(httplib2.Http()))


def resumable_upload(insert_request):
    response = None
    error = None
    retry = 0

    while response is None:
        try:
            print("Uploading file...")
            status, response = insert_request.next_chunk()
            if response is not None:
                if "id" in response:
                    print(f"Video id '{response['id']}' was successfully uploaded.")
                    return response["id"]
                else:
                    raise Exception(f"The upload failed with an unexpected response: {response}")
        except HttpError as e:
            if e.resp.status in RETRIABLE_STATUS_CODES:
                error = f"A retriable HTTP error {e.resp.status} occurred:\n{e.content}"
            else:
                raise
        except RETRIABLE_EXCEPTIONS as e:
            error = f"A retriable error occurred: {e}"

        if error:
            print(error)
            retry += 1
            if retry > MAX_RETRIES:
                raise Exception("No longer attempting to retry.")

            max_sleep = 2**retry
            sleep_seconds = random.random() * max_sleep
            print(f"Sleeping {sleep_seconds:.2f} seconds and then retrying...")
            time.sleep(sleep_seconds)


def upload_video(
    file_path,
    title="Test Title",
    description=DESCRIPTION,
    category="28",
    keywords="Robotics, RSL, ETH, Legged Robotics",
    privacy_status="unlisted",
):
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"File does not exist: {file_path}")

    if privacy_status not in VALID_PRIVACY_STATUSES:
        raise ValueError(f"Invalid privacy status. Must be one of {VALID_PRIVACY_STATUSES}")

    youtube = get_authenticated_service()

    tags = keywords.split(",") if keywords else None

    body = {
        "snippet": {"title": title, "description": description, "tags": tags, "categoryId": category},
        "status": {"privacyStatus": privacy_status},
    }

    media_body = MediaFileUpload(file_path, chunksize=-1, resumable=True)

    insert_request = youtube.videos().insert(
        part=",".join(body.keys()),
        body=body,
        media_body=media_body,
    )

    return resumable_upload(insert_request)


if __name__ == "__main__":
    # Example usage
    video_id = upload_video(
        file_path="/data/GrandTour/2024-10-01-11-29-55/youtube/2024-10-01-11-29-55.mp4",
        title="Debug3",
    )
    print(f"Uploaded video ID: {video_id}")
