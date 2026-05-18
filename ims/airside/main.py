def recv_exact(): 
    """
    Receives exact num_bytes from the socket, unless the connection closes or times out."
    """

def request_image(): 
    """Connects to the transmitter, sends a capture command"""

def load_db(): 
    """Loads the image database from disk"""

def save_db(): 
    """Saves the image database to disk"""

@app.route("/api/ackme")
def ack(): 
    """A simple endpoint to test connectivity."""
    return {"message": "test"}

@app.route("/api/capture_image", methods = ["POST"])
def capture_image(): 
    """Captures an image from the camera and stores it in the database."""

