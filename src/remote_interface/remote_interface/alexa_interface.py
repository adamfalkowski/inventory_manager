import rclpy, yaml, os, numpy as np, cv2 as cv
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from message_interfaces.msg import ItemColor
from flask import Flask, request, jsonify
from ask_sdk_core.skill_builder import SkillBuilder
from flask_ask_sdk.skill_adapter import SkillAdapter
from ask_sdk_core.dispatch_components import AbstractRequestHandler
from ask_sdk_core.utils import is_request_type, is_intent_name
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_model import Response
from ask_sdk_model.ui import SimpleCard
from ask_sdk_core.dispatch_components import AbstractExceptionHandler
from rclpy.node import Node
import rclpy
from threading import Thread


class RemoteSubscriber(Node):
    def __init__(self):
        super().__init__("remote_subscriber")
        self.color_data = {}

        self.subscriber = self.create_subscription(ItemColor, 'item/color', self.subscription_callback, 10)

    def subscription_callback(self, msg):
        self.color_data[msg.color]= {
            "count" : msg.item_count
        }

        self.get_logger().info(f"Updated {msg.color}: {self.color_data[msg.color]}")

    def get_colors(self):
        return self.color_data


# New instance of the Flask class
app = Flask(__name__)


#region Launch Request
class LaunchRequestHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_request_type("LaunchRequest")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Hi, inventory manager activate!"


        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Hello World", speech_text)).set_should_end_session(
            False)
        
        return handler_input.response_builder.response
#endregion

#region Color Intent
class ColorIntentHandler(AbstractRequestHandler):
    def __init__(self, remote_subscriber):
        self.remote_subscriber = remote_subscriber

    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("ColorIntent")(handler_input)

    def handle(self, handler_input):

        color_data = self.remote_subscriber.get_colors()
        # type: (HandlerInput) -> Response

        if not color_data:
            speech_text = "I couldn't retrieve any inventory data right now."
        else:
            parts = []
            for color, data in color_data.items():
                if data["count"] == 0:
                    parts.append(f"no {color} items")
                elif data["count"] == 1:
                    parts.append(f"1 {color} item")
                else:
                    parts.append(f"{data["count"]} {color} items")
            speech_text = "I see " + ", ".join(parts) + "."

        

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Color", speech_text)).set_should_end_session(
            False)

        return handler_input.response_builder.response
#endregion 


def main():
    rclpy.init()
    remote_subscriber = RemoteSubscriber()

    # New instance of the Flask class
    app = Flask(__name__)

    # !New Alexa skills object! that will store all the handlers associated with the skill
    skill_builder = SkillBuilder()
    # add a request handler to the skill, the handler tells how alexa should respomd 
    skill_builder.add_request_handler(LaunchRequestHandler())
    skill_builder.add_request_handler(ColorIntentHandler(remote_subscriber))


    # Register your intent handlers to the skill_builder object

    skill_adapter = SkillAdapter(
        skill=skill_builder.create(), skill_id="amzn1.ask.skill.00b22187-ae06-4319-8b87-ba14c30d11c1", app=app)


    #def alexa_skill():
    #    skill_response = skill_builder.lambda_handler()(request.get_json(),None)
    #    return jsonify(skill_response)
    @app.route("/")
    def invoke_skill():
        return skill_adapter.dispatch_request()

    # Register the web server with the alexa ASK
    skill_adapter.register(app=app, route="/")

    flask_thread = Thread(target=app.run,kwargs={"debug":False,"port":5000},daemon=True)
    flask_thread.start()

    # Spin ROS node
    rclpy.spin(remote_subscriber)
    cv.destroyAllWindows()
    # If you press CTRL+C then you will stop the spin and then you can destroy the node
    remote_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

