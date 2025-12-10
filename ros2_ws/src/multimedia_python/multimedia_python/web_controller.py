from enum import Enum
from threading import Thread

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

app = FastAPI(title="Simple Command API")

# Allow requests from any origin (for testing)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],        # in production you can restrict this
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


class Command(str, Enum):
    PLAY_STOP = "PLAY_STOP"
    SWITCH = "SWITCH"
    NEXT = "NEXT"
    PREV = "PREV"
    SET_VOLUME = "SET_VOLUME"


class CommandRequest(BaseModel):
    command: Command
    value: float | None = None   # Only used for SET_VOLUME


# Will be set to the ROS node instance in main()
ros_node: Node | None = None


@app.post("/command")
async def send_command(request: CommandRequest):
    # Validate SET_VOLUME requirement
    if request.command == Command.SET_VOLUME:
        if request.value is None:
            raise HTTPException(
                status_code=400,
                detail="SET_VOLUME requires a 'value' field."
            )
        if not (0 <= request.value <= 100):
            raise HTTPException(
                status_code=400,
                detail="Volume must be between 0.0 and 1.0."
            )

    # If we have a ROS node, publish the command
    if ros_node is not None and isinstance(ros_node, CommandApiNode):
        if request.command == Command.SET_VOLUME:
            msg = Float32()
            msg.data = request.value / 100
            ros_node.volume_publisher_.publish(msg.data)
        else:
            msg = String()
            msg.data = request.command.value
            ros_node.command_publisher_.publish(msg)
    else:
        print(
            f"[WARN] ROS node not ready, received: "
            f"{request.command} (value={request.value})"
        )

    return {
        "status": "OK",
        "command": request.command,
        "value": request.value,
    }


class CommandApiNode(Node):
    def __init__(self):
        super().__init__("command_api_node")
        self.command_publisher_ = self.create_publisher(
            String,
            '/remote/command_string',
            10,
        )
        self.volume_publisher_ = self.create_publisher(
            Float32,
            '/web_remote/volume_float32',
            10,
        )
        self.get_logger().info("CommandApiNode started")

    def start_api_server(self, host: str = "0.0.0.0", port: int = 8000):
        """
        Start FastAPI/uvicorn in a background thread.
        """
        self.get_logger().info(
            f"Starting FastAPI server on http://{host}:{port}"
        )

        config = uvicorn.Config(
            app,
            host=host,
            port=port,
            log_level="info",
        )
        server = uvicorn.Server(config)

        thread = Thread(target=server.run, daemon=True)
        thread.start()


def main(args=None):
    global ros_node

    rclpy.init(args=args)
    node = CommandApiNode()
    ros_node = node  # make node visible to FastAPI handlers

    # Start FastAPI server in background
    node.start_api_server()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down (KeyboardInterrupt)")
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
