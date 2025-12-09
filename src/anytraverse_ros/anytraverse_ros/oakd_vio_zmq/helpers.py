def get_zmq_uri(name: str) -> str:
    return f"ipc:///tmp/{name}"
