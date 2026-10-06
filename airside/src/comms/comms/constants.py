"""Settings for the link to the AEAC competition server."""

# Environment variable holding the team's connection token.
CONNECTION_TOKEN_ENV = "AEAC_CONNECTION_TOKEN"

# Environment variable that overrides the server's WebSocket URL.
WEBSOCKET_URL_ENV = "AEAC_WEBSOCKET_URL"
DEFAULT_WEBSOCKET_URL = "wss://o61e21rvtd.execute-api.ca-central-1.amazonaws.com/prod"

# Give up on opening a connection after this many seconds.
CONNECT_TIMEOUT_S = 10.0

# Traffic is pushed at 1 Hz. A connection that stays silent for this many
# seconds is treated as dead and reopened.
RECEIVE_TIMEOUT_S = 10.0

# Wait this many seconds before reconnecting after a connection is lost.
RECONNECT_DELAY_S = 2.0

# How often (Hz) traffic received from the server is handed on to ROS.
PUBLISH_POLL_HZ = 10.0
