#!/usr/bin/env python3

import json
import threading

from action_msgs.msg import GoalStatus
from gemini_bridge_interfaces.srv import GetGeminiResponse
from llm_router_msgs.srv import GetLLMResponse
from llama_msgs.action import GenerateResponse
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from type_description_interfaces.msg import KeyValue

AVAIL_MODES = ['local', 'remote-gemini']

# OUTPUT SCHEMA

REQUEST_OUTPUT_SCHEMA_TYPES = {
  'single_types' : [
    GetLLMResponse.Request.INT,
    GetLLMResponse.Request.STRING,
    GetLLMResponse.Request.FLOAT,
    GetLLMResponse.Request.BOOL
  ],
  'array_types' : [
    GetLLMResponse.Request.INT_ARRAY,
    GetLLMResponse.Request.STRING_ARRAY,
    GetLLMResponse.Request.FLOAT_ARRAY,
    GetLLMResponse.Request.BOOL_ARRAY
  ]
}

OUTPUT_SCHEMA_TYPES_FOR_MODE = {
  'local' : {
    'single' : {
      GetLLMResponse.Request.INT : 'integer',
      GetLLMResponse.Request.STRING : 'string',
      GetLLMResponse.Request.FLOAT : 'number',
      GetLLMResponse.Request.BOOL : 'boolean'
    },
    'array' : {
      GetLLMResponse.Request.INT_ARRAY : 'integer',
      GetLLMResponse.Request.STRING_ARRAY : 'string',
      GetLLMResponse.Request.FLOAT_ARRAY : 'number',
      GetLLMResponse.Request.BOOL_ARRAY : 'boolean'
    },
  },
  'remote-gemini' : {
    'single' : {
      GetLLMResponse.Request.INT : 'INTEGER',
      GetLLMResponse.Request.STRING : 'STRING',
      GetLLMResponse.Request.FLOAT : 'NUMBER',
      GetLLMResponse.Request.BOOL : 'BOOLEAN'
    },
    'array' : {
      GetLLMResponse.Request.INT_ARRAY : 'INTEGER',
      GetLLMResponse.Request.STRING_ARRAY : 'STRING',
      GetLLMResponse.Request.FLOAT_ARRAY : 'NUMBER',
      GetLLMResponse.Request.BOOL_ARRAY : 'BOOLEAN'
    },
  }
}

SPECIAL_WORDS_OUTPUT_SCHEMA_FOR_MODE = {
  'array' : {
    'local' : 'array',
    'remote-gemini' : 'ARRAY'
  },
  'object' : {
    'local' : 'object',
    'remote-gemini' : 'OBJECT'
  }
}

class LLMRouter(Node):

  def __init__(self):
    super().__init__('llm_router')

    # PARAMETERS
    self.declare_parameter('mode', 'local')
    self.mode = self.get_parameter('mode').value

    # Parameter validation
    if self.mode not in AVAIL_MODES:
      raise ValueError(
        f'Invalid mode "{self.mode}". Available modes: {AVAIL_MODES}')

    self.ask_llm_callback_group_ = MutuallyExclusiveCallbackGroup()
    self.client_callback_group_ = MutuallyExclusiveCallbackGroup()

    # CLIENTS
    if self.mode == 'local':
      self.llama_client = ActionClient(
        self,
        GenerateResponse,
        '/llama/generate_response',
        callback_group=self.client_callback_group_)

    if self.mode == 'remote-gemini':
      self.gemini_client = self.create_client(
        GetGeminiResponse,
        '/gemini_bridge_service',
        callback_group=self.client_callback_group_)

    # SERVERS
    self.ask_llm_service = self.create_service(
      GetLLMResponse,
      '/ask_llm',
      self.ask_llm_callback,
      callback_group=self.ask_llm_callback_group_)

    # THREADING
    self.llm_response_event = threading.Event()
    self.llm_response_outputs = []
    self.llm_response_error = None

    self.get_logger().info(f'LLM router ready in mode "{self.mode}"')

  def ask_llm_callback(
    self,
    request,
    response):

    self.get_logger().info(f"RECEIVED REQUEST. (mode: {self.mode})")

    self.llm_response_event.clear()
    self.llm_response_outputs = []
    self.llm_response_error = None

    try:
      if self.mode == 'local':
        self.make_llama_request(request)
        
      if self.mode == 'remote-gemini':
        self.make_gemini_request(request)

      self.llm_response_event.wait()

      if self.llm_response_error is not None:
        raise self.llm_response_error

      response.outputs = self.llm_response_outputs
    except (
      RuntimeError,
      ValueError,
      TypeError,
      json.JSONDecodeError,
    ) as exc:
      self.get_logger().error(f'LLM request failed: {exc}')
      response.outputs = []

    self.get_logger().info("SENDING RESPONSE.")

    return response

  # -------- OUTPUT AND RESPONSE --------
  def _build_output_field(
    self,
    mode : str,
    field_type : int
  ): 
    if field_type in REQUEST_OUTPUT_SCHEMA_TYPES['single_types']:
      return {
        'type' : OUTPUT_SCHEMA_TYPES_FOR_MODE[mode]['single'][field_type]
      }
    
    if field_type in REQUEST_OUTPUT_SCHEMA_TYPES['array_types']:
      return {
        'type' : SPECIAL_WORDS_OUTPUT_SCHEMA_FOR_MODE['array'][mode],
        'items' : {
          'type' : OUTPUT_SCHEMA_TYPES_FOR_MODE[mode]['array'][field_type]
        }
      }

    raise ValueError(f'Unsupported output field type "{field_type}"')

  def _build_output_fields_json(
      self,
      mode,
      output_fields
  ):
    properties = {}
    required = []

    for output_field in output_fields:
      properties[output_field.key] = self._build_output_field(mode, output_field.value)
      required.append(output_field.key)
    
    schema = {
      'type' : SPECIAL_WORDS_OUTPUT_SCHEMA_FOR_MODE['object'][mode],
      'properties' : properties,
      'required' : required
    }

    return json.dumps(schema, indent=2)

  def _stringify_output_value(
    self,
    value):
    if isinstance(value, list):
      return json.dumps(value)

    if isinstance(value, dict):
      return json.dumps(value)

    if isinstance(value, bool):
      return 'true' if value else 'false'

    return str(value)

  def _parse_llm_response(
    self,
    response_text
  ):
    parsed_response = json.loads(response_text)

    if isinstance(parsed_response, list):
      if len(parsed_response) == 0:
        return []
      parsed_response = parsed_response[0]

    if not isinstance(parsed_response, dict):
      raise ValueError(
        'Response text must be a JSON object '
        'or a list with one object')

    outputs = []

    for key, value in parsed_response.items():
      output = KeyValue()
      output.key = str(key)
      output.value = self._stringify_output_value(value)
      outputs.append(output)

    return outputs

  # -------- LLAMA-ROS ---------

  def make_llama_request(
    self,
    llm_request : GetLLMResponse.Request
  ):
    if not self.llama_client.wait_for_server(1):
      raise RuntimeError('Llama response generation action is not available')
    
    llama_request = GenerateResponse.Goal()

    llama_request.prompt = llm_request.prompt
    llama_request.sampling_config.grammar_schema = self._build_output_fields_json(
      'local', llm_request.output_fields)

    if llm_request.uses_image:
      llama_request.prompt = "Look at the image: <__media__> \n" + llama_request.prompt
      llama_request.images.append(llm_request.image)

    self.get_logger().info(f"Prompt: {llama_request.prompt}")
    self.get_logger().info(f"Grammar schema: {llama_request.sampling_config.grammar_schema}")

    llama_future = self.llama_client.send_goal_async(llama_request)
    llama_future.add_done_callback(self._llama_goal_response_callback)

    return llama_future

  def _llama_goal_response_callback(
    self,
    future
  ):
    try:
      goal_handle = future.result()

      if not goal_handle.accepted:
        raise RuntimeError('Llama response generation goal was rejected')

      result_future = goal_handle.get_result_async()
      result_future.add_done_callback(self._llama_response_callback)
    except RuntimeError as exc:
      self.llm_response_outputs = []
      self.llm_response_error = exc
      self.get_logger().error(f'Failed to process LLM response: {exc}')
      self.llm_response_event.set()

  def _llama_response_callback(
    self,
    future
  ):
    try:
      llama_result = future.result()

      if llama_result.status != GoalStatus.STATUS_SUCCEEDED:
        raise RuntimeError(
          'Llama response generation action not succeeded with '
          f'status code {llama_result.status}')

      self.llm_response_outputs = self._parse_llm_response(
        llama_result.result.response.text)
      self.llm_response_error = None
    except (
      RuntimeError,
      ValueError,
      TypeError,
      json.JSONDecodeError,
    ) as exc:
      self.llm_response_outputs = []
      self.llm_response_error = exc
      self.get_logger().error(f'Failed to process LLM response: {exc}')
    finally:
      self.llm_response_event.set()

  # -------- GEMINI ------------

  def make_gemini_request(
    self,
    llm_request : GetLLMResponse.Request
  ):
    if not self.gemini_client.service_is_ready():
      raise RuntimeError('Gemini bridge service is not available')

    gemini_request = GetGeminiResponse.Request()
    gemini_request.prompt = llm_request.prompt
    gemini_request.uses_image = llm_request.uses_image
    gemini_request.output_fields_json = self._build_output_fields_json(
      'remote-gemini', llm_request.output_fields)

    if llm_request.uses_image:
      gemini_request.img = llm_request.image

    gemini_future = self.gemini_client.call_async(gemini_request)
    gemini_future.add_done_callback(self._gemini_response_callback)

    return gemini_future

  def _gemini_response_callback(
    self,
    future : GetGeminiResponse.Response
  ):
    try:
      gemini_response = future.result()
      self.llm_response_outputs = self._parse_llm_response(
        gemini_response.gemini_response)
      self.llm_response_error = None
    except (
      RuntimeError,
      ValueError,
      TypeError,
      json.JSONDecodeError,
    ) as exc:
      self.llm_response_outputs = []
      self.llm_response_error = exc
      self.get_logger().error(f'Failed to process LLM response: {exc}')
    finally:
      self.llm_response_event.set()


def main(args=None):
  rclpy.init(args=args)

  node = LLMRouter()
  executor = MultiThreadedExecutor()
  executor.add_node(node)

  try:
    executor.spin()
  except KeyboardInterrupt:
    pass
  finally:
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()
