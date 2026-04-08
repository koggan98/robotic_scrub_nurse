#!/usr/bin/env python3
"""
LLM Reasoning Node
==================
Processes natural language commands from the surgeon, queries the world model,
and issues execution plans via OpenAI tool calling.

Pipeline:
  /user_speech (String) → LLM with tool calling
    → get_world_state() (service call to world model)
    → get_tool_candidates(class) (service call to world model)
    → decide action + parameters
    → publish ExecutionPlan
    → publish /system_response (String) for TTS feedback

Subscriptions:
  /user_speech (std_msgs/String) - transcribed speech from ASR node

Publishers:
  /execution_plan (tracking_pkg/msg/ExecutionPlan)
  /system_response (std_msgs/String) - feedback text for TTS node

Service Clients:
  /get_world_state (GetWorldState)
  /get_tool_candidates (GetToolCandidates)

Parameters:
  openai_api_key (str): OpenAI API key (or set OPENAI_API_KEY env var)
  model_name (str): OpenAI model to use (default: gpt-4o)
  knowledge_base_path (str): Path to tool_knowledge_base.yaml for system prompt
"""

import os
import json
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

# from tracking_pkg.msg import ExecutionPlan
# from tracking_pkg.srv import GetWorldState, GetToolCandidates


class LLMReasoningNode(Node):
    def __init__(self):
        super().__init__('llm_reasoning_node')

        # Parameters
        self.declare_parameter('openai_api_key', '')
        self.declare_parameter('model_name', 'gpt-4o')
        self.declare_parameter('knowledge_base_path', '')

        api_key = self.get_parameter('openai_api_key').value
        if not api_key:
            api_key = os.environ.get('OPENAI_API_KEY', '')
        self.model_name = self.get_parameter('model_name').value

        # Load knowledge base for system prompt
        kb_path = self.get_parameter('knowledge_base_path').value
        if not kb_path:
            pkg_share = get_package_share_directory('tracking_pkg')
            kb_path = os.path.join(pkg_share, 'config', 'tool_knowledge_base.yaml')
        self.knowledge_base = self._load_knowledge_base(kb_path)

        # OpenAI client (lazy init)
        self._client = None
        self._api_key = api_key

        # Subscribers
        self.create_subscription(String, 'user_speech', self._speech_cb, 10)

        # Publishers
        # self.plan_pub = self.create_publisher(ExecutionPlan, 'execution_plan', 10)
        self.response_pub = self.create_publisher(String, 'system_response', 10)

        # Service clients
        # self.world_state_client = self.create_client(GetWorldState, 'get_world_state')
        # self.tool_candidates_client = self.create_client(GetToolCandidates, 'get_tool_candidates')

        self.get_logger().info('LLMReasoningNode initialized')

    def _load_knowledge_base(self, path):
        try:
            with open(path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().warn(f'Could not load knowledge base: {e}')
            return {}

    def _get_openai_client(self):
        if self._client is None:
            try:
                from openai import OpenAI
                self._client = OpenAI(api_key=self._api_key)
            except ImportError:
                self.get_logger().error('openai package not installed. pip install openai')
                return None
            except Exception as e:
                self.get_logger().error(f'Failed to create OpenAI client: {e}')
                return None
        return self._client

    def _build_system_prompt(self):
        """Build system prompt with tool knowledge base context."""
        tools_desc = ""
        for cls_name, info in self.knowledge_base.get('tools', {}).items():
            synonyms = ', '.join(info.get('synonyms', []))
            tools_desc += (
                f"- {info.get('display_name', cls_name)} "
                f"(class: {cls_name}, synonyms: {synonyms}): "
                f"{info.get('description', '')} "
                f"Handover: {info.get('handover_description', '')}\n"
            )

        return f"""You are the reasoning system of a robotic scrub nurse in an operating room.
Your role is to understand the surgeon's natural language requests for surgical instruments,
identify the correct tool, and coordinate the robot to pick it up and hand it over safely.

Available instruments on the tray:
{tools_desc}

You have access to the following functions:
- get_world_state(): Returns the current system state including available tools, hand position, and robot status.
- get_tool_candidates(tool_class): Returns detected tools matching a class name.
- execute_pick_and_handover(tool_id, handover_rule): Commands the robot to pick a tool and hand it to the surgeon.
- abort(): Emergency stop.

Always:
1. First call get_world_state() to understand the current situation.
2. If the surgeon requests a tool, call get_tool_candidates() with the matching class.
3. If multiple candidates match, ask the surgeon to clarify (respond via text).
4. Once a unique tool is identified, call execute_pick_and_handover().
5. Respond in the same language the surgeon uses (German or English).
"""

    def _get_tool_definitions(self):
        """OpenAI function/tool definitions for tool calling."""
        return [
            {
                "type": "function",
                "function": {
                    "name": "get_world_state",
                    "description": "Get the current world model snapshot including system state, available tools, hand position, and robot status.",
                    "parameters": {"type": "object", "properties": {}, "required": []}
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "get_tool_candidates",
                    "description": "Get detected tools on the tray, optionally filtered by class name.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "tool_class": {
                                "type": "string",
                                "description": "Tool class to filter by (e.g. 'needle_holder', 'scalpel'). Empty for all."
                            }
                        },
                        "required": ["tool_class"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "execute_pick_and_handover",
                    "description": "Command the robot to pick up a specific tool and hand it to the surgeon.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "tool_id": {
                                "type": "string",
                                "description": "The unique tool ID from the world model."
                            },
                            "handover_rule": {
                                "type": "string",
                                "enum": ["functional_end_away", "blade_away", "neutral"],
                                "description": "How to orient the tool during handover."
                            }
                        },
                        "required": ["tool_id", "handover_rule"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "abort",
                    "description": "Emergency stop. Halt all robot motion immediately.",
                    "parameters": {"type": "object", "properties": {}, "required": []}
                }
            }
        ]

    def _speech_cb(self, msg):
        """Handle incoming transcribed speech."""
        text = msg.data.strip()
        if not text:
            return

        self.get_logger().info(f'User speech: "{text}"')

        client = self._get_openai_client()
        if client is None:
            self._publish_response("System error: LLM not available.")
            return

        # TODO: Implement full tool-calling loop
        # 1. Send user text + system prompt + tool definitions to OpenAI
        # 2. Process tool calls in a loop:
        #    - get_world_state → call /get_world_state service, return result
        #    - get_tool_candidates → call /get_tool_candidates service, return result
        #    - execute_pick_and_handover → publish ExecutionPlan, return confirmation
        #    - abort → publish abort ExecutionPlan
        # 3. Final assistant message → publish as /system_response
        self.get_logger().info('LLM processing not yet implemented')

    def _publish_response(self, text):
        """Publish text response for TTS node."""
        msg = String()
        msg.data = text
        self.response_pub.publish(msg)
        self.get_logger().info(f'Response: "{text}"')


def main(args=None):
    rclpy.init(args=args)
    node = LLMReasoningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
