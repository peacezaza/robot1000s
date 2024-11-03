import pygame
import websocket
import json

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((800, 800))
pygame.display.set_caption("Robot Mapping")

# Initial position
x, y = 400, 400
current_direction = "East"  # Start with initial direction as East

# Direction vectors
directions = {
    "North": (0, -1),
    "South": (0, 1),
    "East": (1, 0),
    "West": (-1, 0)
}

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)


# Draw a line based on the direction and distance (time)
def draw_line(direction, time):
    global x, y

    # Calculate movement vector
    dx, dy = directions[direction]
    new_x = x + dx * time * 10  # Scale factor to increase line length
    new_y = y + dy * time * 10

    # Draw line from (x, y) to (new_x, new_y)
    pygame.draw.line(screen, BLACK, (x, y), (new_x, new_y), 2)
    pygame.display.flip()

    # Update position
    x, y = new_x, new_y


# WebSocket event handlers
def on_message(ws, message):
    global current_direction
    data = json.loads(message)

    # Update direction and draw the line
    direction = data["direction"]
    time = data["time"]

    # Draw the line on the map
    draw_line(direction, time)
    current_direction = direction
    print("Received: ", message)


def on_error(ws, error):
    print("Error:", error)


def on_close(ws, close_status_code, close_msg):
    print("Connection closed")


def on_open(ws):
    print("Connected to server")


# Set up the WebSocket
ws_url = "ws://20.2.250.248:1880/ws/data"
ws = websocket.WebSocketApp(
    ws_url,
    on_open=on_open,
    on_message=on_message,
    on_error=on_error,
    on_close=on_close
)

# Run the WebSocket in a thread
import threading

ws_thread = threading.Thread(target=ws.run_forever)
ws_thread.daemon = True
ws_thread.start()

# Main Pygame loop
running = True
screen.fill(WHITE)
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Update display
    pygame.display.flip()

pygame.quit()
