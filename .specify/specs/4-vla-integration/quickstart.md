# Quickstart Guide: Vision-Language-Action (VLA) Integration

## Overview
This guide provides a step-by-step introduction to implementing Vision-Language-Action (VLA) integration for humanoid robotics. By following this guide, you'll understand how to connect voice recognition, language models, and robotic action in a cohesive system.

## Prerequisites
- ROS 2 Humble Hawksbill installed and configured
- Python 3.8+ with pip
- OpenAI API key for Whisper and LLM services
- Audio input device (microphone) for voice commands
- Humanoid robot simulation environment (or real robot)
- Basic understanding of ROS 2 concepts

## Setting Up the Environment

### 1. Install Required Dependencies
```bash
# Install ROS 2 dependencies
sudo apt update
sudo apt install python3-rosdep python3-colcon-common-extensions

# Install Python packages
pip install openai speech-recognition rclpy numpy transforms3d
```

### 2. Configure API Access
```bash
# Set your OpenAI API key
export OPENAI_API_KEY="your-api-key-here"

# Verify API access
python -c "import openai; print(openai.Model.list())"
```

### 3. Set Up Audio Environment
```bash
# Test audio input
python -c "import speech_recognition as sr; r = sr.Recognizer(); m = sr.Microphone(); print('Microphone found:', m)"
```

## Voice-to-Action Pipeline

### 1. Basic Voice Recognition
Create a simple voice command recognizer:

```python
import openai
import speech_recognition as sr
import rclpy
from rclpy.node import Node

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Adjust for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

    def listen_for_command(self):
        """Listen for a voice command and return transcribed text"""
        with self.microphone as source:
            self.get_logger().info("Listening for command...")
            audio = self.recognizer.listen(source, timeout=5.0)

        try:
            # Use Whisper API for transcription
            transcript = openai.Audio.transcribe(
                "whisper-1",
                audio,
                response_format="verbose_json"
            )
            return transcript.text
        except Exception as e:
            self.get_logger().error(f"Transcription error: {e}")
            return None

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()

    command = node.listen_for_command()
    if command:
        print(f"Recognized command: {command}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Processing Voice Commands
```python
class VLAProcessor:
    def __init__(self):
        self.command_history = []

    def process_command(self, text_command):
        """Process natural language command and return action plan"""
        # Use LLM to interpret the command
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[
                {
                    "role": "system",
                    "content": "You are a robotic command interpreter. Convert natural language commands into structured robot actions. Respond with JSON containing action_type and parameters."
                },
                {
                    "role": "user",
                    "content": f"Convert this command to robot actions: '{text_command}'. Robot capabilities: navigation, object manipulation, perception."
                }
            ],
            response_format={"type": "json_object"}
        )

        action_plan = response.choices[0].message.content
        return action_plan
```

## Cognitive Planning with LLMs

### 1. Task Decomposition
```python
class TaskPlanner:
    def __init__(self):
        self.robot_capabilities = [
            "navigation",
            "manipulation",
            "perception",
            "communication"
        ]

    def decompose_task(self, natural_language_command):
        """Decompose complex commands into executable steps"""
        prompt = f"""
        Decompose this command into specific robotic actions: "{natural_language_command}"

        Available capabilities: {', '.join(self.robot_capabilities)}

        Provide the response as JSON with:
        - action_sequence: array of action objects
        - each action has: action_type, parameters, dependencies

        Example format:
        {{
            "action_sequence": [
                {{"action_type": "perception", "parameters": {{"target": "kitchen"}}, "dependencies": []}},
                {{"action_type": "navigation", "parameters": {{"target_location": "kitchen"}}, "dependencies": ["0"]}},
                {{"action_type": "manipulation", "parameters": {{"object": "cup", "action": "grasp"}}, "dependencies": ["1"]}}
            ]
        }}
        """

        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[{"role": "user", "content": prompt}],
            response_format={"type": "json_object"}
        )

        return response.choices[0].message.content
```

### 2. Action Mapping
```python
class ActionMapper:
    def __init__(self):
        self.action_templates = {
            "navigation": self._execute_navigation,
            "manipulation": self._execute_manipulation,
            "perception": self._execute_perception
        }

    def map_to_ros_action(self, action_definition):
        """Map action definition to ROS 2 action call"""
        action_type = action_definition.get("action_type")
        parameters = action_definition.get("parameters", {})

        if action_type in self.action_templates:
            return self.action_templates[action_type](parameters)
        else:
            raise ValueError(f"Unknown action type: {action_type}")

    def _execute_navigation(self, params):
        """Execute navigation action"""
        # Implementation would use ROS 2 navigation2 stack
        pass

    def _execute_manipulation(self, params):
        """Execute manipulation action"""
        # Implementation would use ROS 2 moveit or similar
        pass

    def _execute_perception(self, params):
        """Execute perception action"""
        # Implementation would use ROS 2 perception pipeline
        pass
```

## Complete VLA Pipeline

### 1. Integration Example
```python
class VLAPipeline:
    def __init__(self):
        self.voice_processor = VoiceCommandNode()
        self.task_planner = TaskPlanner()
        self.action_mapper = ActionMapper()

    def execute_vla_command(self, audio_input=None, text_input=None):
        """Complete VLA pipeline execution"""
        # Step 1: Get command text (from audio or text)
        if audio_input:
            command_text = self.voice_processor.process_audio(audio_input)
        elif text_input:
            command_text = text_input
        else:
            raise ValueError("Either audio_input or text_input must be provided")

        if not command_text:
            return {"success": False, "error": "Could not process command"}

        # Step 2: Plan the task
        task_plan = self.task_planner.decompose_task(command_text)

        # Step 3: Execute the plan
        execution_results = []
        for action in task_plan["action_sequence"]:
            try:
                result = self.action_mapper.map_to_ros_action(action)
                execution_results.append(result)
            except Exception as e:
                return {"success": False, "error": f"Action failed: {str(e)}"}

        return {
            "success": True,
            "command_text": command_text,
            "task_plan": task_plan,
            "execution_results": execution_results
        }
```

### 2. Running the Complete System
```python
def main():
    # Initialize ROS 2
    rclpy.init()

    # Create VLA pipeline
    vla_system = VLAPipeline()

    # Example: Process a voice command
    result = vla_system.execute_vla_command(text_input="Go to the kitchen and bring me a cup")

    if result["success"]:
        print("Command executed successfully!")
        print(f"Original command: {result['command_text']}")
        print(f"Task plan: {result['task_plan']}")
    else:
        print(f"Command failed: {result['error']}")

    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

## Testing Your VLA System

### 1. Unit Tests
```python
import unittest

class TestVLAComponents(unittest.TestCase):
    def test_voice_recognition(self):
        """Test voice recognition component"""
        processor = VoiceCommandNode()
        # Mock audio input and verify transcription
        self.assertIsNotNone(processor)

    def test_task_decomposition(self):
        """Test task decomposition with LLM"""
        planner = TaskPlanner()
        command = "Pick up the red ball"
        plan = planner.decompose_task(command)
        # Verify plan structure
        self.assertIn("action_sequence", plan)
```

### 2. Integration Test
```bash
# Run the complete VLA system
python vla_system.py

# Test with different commands
# "Move forward 1 meter"
# "Turn left and stop"
# "Go to the table and pick up the cup"
```

## Troubleshooting Common Issues

### Audio Input Problems
- **Issue**: No audio input detected
- **Solution**: Check microphone permissions and test with `arecord -l`

### LLM Response Issues
- **Issue**: LLM returns unstructured responses
- **Solution**: Ensure you're using `response_format={"type": "json_object"}`

### ROS 2 Connection Issues
- **Issue**: Cannot connect to robot services
- **Solution**: Verify ROS 2 network configuration and service availability

## Next Steps

1. **Enhance Voice Processing**: Add noise reduction and multiple language support
2. **Improve Planning**: Add more sophisticated task decomposition algorithms
3. **Optimize Performance**: Implement caching and parallel processing
4. **Add Safety Features**: Include safety checks and validation layers
5. **Extend Capabilities**: Add more robot action types and perception modalities

## Resources

- [OpenAI API Documentation](https://platform.openai.com/docs/api-reference)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Whisper API Guide](https://platform.openai.com/docs/guides/speech-to-text)
- [Navigation2 Package](https://navigation.ros.org/)

This quickstart guide provides the foundation for building more sophisticated VLA systems. The next steps involve expanding each component and integrating them into a robust, production-ready system.