---
sidebar_position: 3
title: "Cognitive Planning with LLMs: Natural Language to ROS 2 Actions"
description: "Using Large Language Models for natural language understanding and task decomposition in robotics"
---

# Cognitive Planning with LLMs: Natural Language to ROS 2 Actions

## Learning Objectives

By the end of this chapter, you will:
- Understand the fundamentals of Large Language Models (LLMs) in robotics applications
- Learn techniques for natural language understanding specifically for robotics
- Master task decomposition strategies for complex robotic missions
- Implement mapping from natural language to ROS 2 actions
- Handle ambiguous or complex commands with fallback strategies

## Introduction to LLM-Based Cognitive Planning

Cognitive planning represents the intelligence layer in Vision-Language-Action (VLA) systems, where natural language commands are transformed into executable robotic actions. Large Language Models (LLMs) excel at understanding the nuances of human language and can decompose complex tasks into simpler, actionable steps.

In humanoid robotics, cognitive planning bridges the gap between high-level human intentions and low-level robot control. When a human says "Please bring me the coffee from the kitchen," the LLM must understand the components of this task: navigation to the kitchen, object recognition to identify the coffee, manipulation to grasp it, and navigation back to the human.

## Large Language Models in Robotics

LLMs bring several advantages to robotic applications:

- **Natural Language Understanding**: Ability to interpret complex, ambiguous, or context-dependent commands
- **Knowledge Integration**: Access to world knowledge that can inform planning decisions
- **Task Decomposition**: Breaking complex goals into sequences of simpler actions
- **Adaptability**: Handling novel situations by reasoning about similar scenarios

### Key Considerations for LLMs in Robotics

1. **Latency Requirements**: Real-time robotic applications may require response times that challenge some LLM implementations
2. **Reliability**: Robotic safety requires predictable and reliable planning, which can be challenging with probabilistic models
3. **Domain Adaptation**: General LLMs may need fine-tuning or prompting strategies for robotics-specific tasks
4. **Embodiment**: LLMs must understand the physical constraints and capabilities of the robotic platform

## Natural Language Understanding for Robotics

Natural Language Understanding (NLU) in robotics differs from general NLU in several key ways:

### Spatial Reasoning

Robots must understand spatial relationships and locations:

```llm-codeblock
type: planning
title: "Spatial Reasoning Example"
description: "Processing spatial relationships in natural language"
```
```python
def extract_spatial_relationships(command):
    """Extract spatial relationships from natural language command"""
    spatial_patterns = {
        'relative_position': [
            r'(?P<action>\w+)\s+to\s+(?P<reference>\w+)',
            r'(?P<action>\w+)\s+(?P<direction>\w+)\s+of\s+(?P<reference>\w+)'
        ],
        'absolute_position': [
            r'at\s+(?P<location>[\w\s]+)',
            r'in\s+(?P<location>[\w\s]+)',
            r'on\s+(?P<location>[\w\s]+)'
        ]
    }

    # Process command with spatial patterns
    # Return structured spatial information
    pass
```
```

### Action Recognition

Identifying specific robotic actions from natural language:

```llm-codeblock
type: task-decomposition
title: "Action Recognition"
description: "Identifying robotic actions from natural language"
```
```python
def recognize_actions(command, action_vocabulary):
    """Recognize specific robotic actions from command"""
    recognized_actions = []

    for action in action_vocabulary:
        if action in command.lower():
            recognized_actions.append(action)

    return recognized_actions

# Example action vocabulary for a humanoid robot
ROBOT_ACTIONS = [
    'navigate', 'grasp', 'release', 'manipulate', 'inspect',
    'approach', 'avoid', 'follow', 'stop', 'wait'
]
```
```

### Context Awareness

Understanding commands in the context of the current environment and task state:

```llm-codeblock
type: planning
title: "Context-Aware Processing"
description: "Processing commands with environmental context"
```
```python
class ContextAwareCommandProcessor:
    def __init__(self, environment_model, robot_state):
        self.environment_model = environment_model
        self.robot_state = robot_state

    def process_command_with_context(self, command):
        """Process command considering environmental context"""
        # Incorporate current robot position, known objects, etc.
        contextual_command = self.add_context(command)
        return self.process_command(contextual_command)

    def add_context(self, command):
        """Add environmental context to command"""
        context = f"Robot position: {self.robot_state.position}, "
        context += f"Known objects: {self.environment_model.objects}, "
        context += f"Command: {command}"
        return context
```
```

## Task Decomposition Strategies

Complex robotic tasks need to be decomposed into simpler, executable actions. Here are several effective strategies:

### Hierarchical Task Networks (HTN)

Break down high-level goals into hierarchies of subtasks:

```llm-codeblock
type: task-decomposition
title: "Hierarchical Task Decomposition"
description: "Breaking complex tasks into hierarchical subtasks"
```
```python
class HierarchicalTaskDecomposer:
    def __init__(self):
        self.task_library = {
            'fetch_object': [
                'navigate_to_location',
                'identify_object',
                'grasp_object',
                'return_to_origin'
            ],
            'navigate_to_location': [
                'plan_path',
                'execute_navigation'
            ],
            'grasp_object': [
                'position_hand',
                'close_gripper',
                'verify_grasp'
            ]
        }

    def decompose_task(self, high_level_task):
        """Decompose high-level task into primitive actions"""
        if high_level_task in self.task_library:
            subtasks = self.task_library[high_level_task]
            return [self.decompose_task(subtask) if subtask in self.task_library
                   else subtask for subtask in subtasks]
        else:
            # Primitive action
            return [high_level_task]
```
```

### Sequential Decomposition

For tasks that must be performed in a specific order:

```llm-codeblock
type: task-decomposition
title: "Sequential Task Decomposition"
description: "Decomposing tasks into sequential steps"
```
```python
def sequential_decomposition(command):
    """Decompose command into sequential actions"""
    # Example: "Go to kitchen, pick up cup, bring to me"
    steps = command.split(',')
    actions = []

    for step in steps:
        action = identify_action(step.strip())
        if action:
            actions.append(action)

    return actions

def identify_action(step):
    """Identify specific action from step description"""
    # Map natural language to ROS 2 actions
    action_mapping = {
        'go to kitchen': 'navigate_to_kitchen',
        'pick up cup': 'grasp_cup',
        'bring to me': 'navigate_to_operator'
    }

    for pattern, action in action_mapping.items():
        if pattern in step.lower():
            return action

    return None
```
```

### Parallel Task Decomposition

For tasks that can be performed simultaneously:

```llm-codeblock
type: task-decomposition
title: "Parallel Task Decomposition"
description: "Identifying tasks that can be performed in parallel"
```
```python
def identify_parallel_tasks(task_list):
    """Identify tasks that can be performed in parallel"""
    # Example: While navigating, the robot can simultaneously look for objects
    parallel_groups = []

    for task in task_list:
        if can_run_parallel(task, task_list):
            parallel_groups.append(task)

    return parallel_groups

def can_run_parallel(task, task_list):
    """Determine if task can run in parallel with others"""
    # Check for resource conflicts
    # Check for dependency relationships
    # Return True if parallel execution is safe
    pass
```
```

## Mapping Natural Language to ROS 2 Actions

The core challenge in cognitive planning is translating natural language into executable ROS 2 actions. This involves several steps:

### Step 1: Intent Recognition

Identify the user's intent from the natural language command:

```llm-codeblock
type: action-mapping
title: "Intent Recognition"
description: "Identifying user intent from natural language"
```
```python
import re

def recognize_intent(command):
    """Recognize intent from natural language command"""
    intent_patterns = {
        'navigation': [
            r'\bgo to\b', r'\bmove to\b', r'\bnavigate to\b',
            r'\bwalk to\b', r'\bhead to\b'
        ],
        'manipulation': [
            r'\bpick up\b', r'\bgrasp\b', r'\btake\b',
            r'\brelease\b', r'\bdrop\b', r'\bput down\b'
        ],
        'inspection': [
            r'\blook at\b', r'\bexamine\b', r'\binspect\b',
            r'\bcheck\b', r'\bfind\b'
        ]
    }

    for intent, patterns in intent_patterns.items():
        for pattern in patterns:
            if re.search(pattern, command, re.IGNORECASE):
                return intent

    return 'unknown'
```
```

### Step 2: Parameter Extraction

Extract specific parameters needed for the action:

```llm-codeblock
type: action-mapping
title: "Parameter Extraction"
description: "Extracting action parameters from natural language"
```
```python
def extract_parameters(command, intent):
    """Extract parameters needed for the identified intent"""
    if intent == 'navigation':
        # Extract destination
        destination_match = re.search(r'to\s+(.+?)(?:\.|$)', command)
        if destination_match:
            return {'destination': destination_match.group(1).strip()}

    elif intent == 'manipulation':
        # Extract object to manipulate
        object_match = re.search(r'(?:pick up|grasp|take|release)\s+(.+?)(?:\.|$)', command)
        if object_match:
            return {'object': object_match.group(1).strip()}

    return {}
```
```

### Step 3: Action Generation

Generate specific ROS 2 actions based on intent and parameters:

```llm-codeblock
type: action-mapping
title: "Action Generation"
description: "Generating ROS 2 actions from intent and parameters"
```
```python
class ROSActionGenerator:
    def __init__(self, ros_node):
        self.ros_node = ros_node

    def generate_action(self, intent, parameters):
        """Generate ROS 2 action based on intent and parameters"""
        if intent == 'navigation':
            return self.create_navigation_action(parameters)
        elif intent == 'manipulation':
            return self.create_manipulation_action(parameters)
        elif intent == 'inspection':
            return self.create_inspection_action(parameters)
        else:
            return None

    def create_navigation_action(self, parameters):
        """Create ROS 2 navigation action"""
        # Example: Create a navigation goal
        goal = {
            'target_pose': self.get_pose_for_location(parameters['destination'])
        }
        return goal

    def create_manipulation_action(self, parameters):
        """Create ROS 2 manipulation action"""
        # Example: Create a manipulation goal
        goal = {
            'object_name': parameters['object'],
            'action_type': 'grasp' if 'pick' in parameters.get('original_command', '') else 'release'
        }
        return goal

    def get_pose_for_location(self, location_name):
        """Get pose for a known location name"""
        # Map location name to known pose
        location_poses = {
            'kitchen': {'x': 1.0, 'y': 2.0, 'z': 0.0},
            'living room': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'bedroom': {'x': -1.0, 'y': 1.0, 'z': 0.0}
        }
        return location_poses.get(location_name.lower(), {'x': 0.0, 'y': 0.0, 'z': 0.0})
```
```

## Handling Ambiguous or Complex Commands

Real-world natural language commands are often ambiguous or complex. Here are strategies to handle these cases:

### Disambiguation Strategies

When commands are unclear, implement strategies to resolve ambiguity:

```llm-codeblock
type: planning
title: "Disambiguation Strategies"
description: "Handling ambiguous commands with clarification requests"
```
```python
def handle_ambiguous_command(command, context):
    """Handle ambiguous commands by seeking clarification"""
    # Identify ambiguity in the command
    ambiguities = identify_ambiguities(command)

    if ambiguities:
        # Generate clarification questions
        questions = generate_clarification_questions(ambiguities, context)
        return {'type': 'clarification', 'questions': questions}
    else:
        # Command is clear, proceed with processing
        return {'type': 'action', 'command': command}

def identify_ambiguities(command):
    """Identify potential ambiguities in the command"""
    # Check for ambiguous references, unclear actions, etc.
    potential_ambiguities = []

    # Example: "Pick up the box" - which box?
    if re.search(r'the\s+(\w+)', command):
        potential_ambiguities.append({
            'type': 'object_reference',
            'object': re.search(r'the\s+(\w+)', command).group(1)
        })

    return potential_ambiguities
```
```

### Fallback Strategies

Implement fallback strategies when LLM planning fails:

```llm-codeblock
type: planning
title: "Fallback Strategies"
description: "Alternative approaches when primary planning fails"
```
```python
class PlanningWithFallbacks:
    def __init__(self, primary_planner, fallback_planners):
        self.primary_planner = primary_planner
        self.fallback_planners = fallback_planners

    def plan_with_fallbacks(self, command):
        """Attempt planning with fallbacks if primary fails"""
        try:
            # Try primary planner
            plan = self.primary_planner.generate_plan(command)
            if plan and self.validate_plan(plan):
                return plan
        except Exception as e:
            print(f"Primary planner failed: {e}")

        # Try fallback planners in order
        for fallback_planner in self.fallback_planners:
            try:
                plan = fallback_planner.generate_plan(command)
                if plan and self.validate_plan(plan):
                    return plan
            except Exception as e:
                print(f"Fallback planner failed: {e}")
                continue

        # If all planners fail, return error
        return {'error': 'Unable to generate plan for command', 'command': command}

    def validate_plan(self, plan):
        """Validate that the generated plan is executable"""
        # Check for required parameters, valid action sequences, etc.
        return plan is not None and len(plan) > 0
```
```

### Multi-Step Planning

For complex commands that require multiple steps:

```llm-codeblock
type: task-decomposition
title: "Multi-Step Planning"
description: "Breaking complex commands into multi-step plans"
```
```python
def generate_multi_step_plan(command):
    """Generate multi-step plan for complex commands"""
    # Use LLM to break down complex command
    steps = llm_break_down_command(command)

    plan = []
    for step in steps:
        intent = recognize_intent(step)
        parameters = extract_parameters(step, intent)
        action = generate_action(intent, parameters)
        plan.append(action)

    return plan

def llm_break_down_command(command):
    """Use LLM to break down complex command into simpler steps"""
    # Example prompt to LLM:
    prompt = f"""
    Break down the following complex command into simpler, sequential steps:
    Command: "{command}"

    Provide the steps as a numbered list:
    1. [First step]
    2. [Second step]
    ...
    """

    # Call LLM with this prompt and return the steps
    # This is a conceptual example - actual implementation would call an LLM API
    return ["navigate to kitchen", "identify coffee mug", "grasp coffee mug", "return to user"]
```
```

## Practical Implementation Examples

Let's look at practical implementations of LLM-based cognitive planning:

### Example 1: Simple Command Processing

A basic implementation for processing simple commands:

```llm-codeblock
type: action-mapping
title: "Simple Command Processing"
description: "Basic implementation of natural language to action mapping"
```
```python
class SimpleCommandProcessor:
    def __init__(self, ros_node):
        self.ros_node = ros_node
        self.action_generator = ROSActionGenerator(ros_node)

    def process_command(self, command):
        """Process a simple command end-to-end"""
        # Step 1: Recognize intent
        intent = recognize_intent(command)

        # Step 2: Extract parameters
        parameters = extract_parameters(command, intent)

        # Step 3: Generate action
        action = self.action_generator.generate_action(intent, parameters)

        # Step 4: Execute action
        if action:
            self.execute_action(action)
            return {'status': 'success', 'action': action}
        else:
            return {'status': 'failure', 'error': 'Could not generate action'}

    def execute_action(self, action):
        """Execute the generated action via ROS 2"""
        # Implementation to send action to ROS 2 action server
        pass
```
```

### Example 2: Complex Task with Context

A more sophisticated implementation that considers context:

```llm-codeblock
type: planning
title: "Context-Aware Task Planning"
description: "Advanced planning considering environmental context"
```
```python
class ContextAwarePlanner:
    def __init__(self, ros_node, environment_model):
        self.ros_node = ros_node
        self.environment_model = environment_model
        self.task_decomposer = HierarchicalTaskDecomposer()

    def plan_task_with_context(self, command):
        """Plan task considering environmental context"""
        # Get current context
        context = {
            'robot_position': self.get_robot_position(),
            'known_objects': self.environment_model.get_known_objects(),
            'recent_actions': self.get_recent_actions()
        }

        # Generate contextual command
        contextual_command = self.add_context_to_command(command, context)

        # Process with LLM to generate plan
        plan = self.generate_plan_with_llm(contextual_command)

        # Validate and refine plan
        validated_plan = self.validate_plan(plan, context)

        return validated_plan

    def add_context_to_command(self, command, context):
        """Add environmental context to the command"""
        contextual_command = f"""
        Context:
        - Robot is currently at position: {context['robot_position']}
        - Known objects in environment: {context['known_objects']}
        - Recent actions: {context['recent_actions']}

        Command: {command}

        Generate a plan considering the above context.
        """
        return contextual_command

    def generate_plan_with_llm(self, contextual_command):
        """Generate plan using LLM with context"""
        # Call LLM with contextual command
        # Return structured plan
        pass

    def validate_plan(self, plan, context):
        """Validate plan against environmental constraints"""
        # Check if plan is feasible given current context
        # Return validated plan or None if invalid
        pass
```
```

## Performance Considerations

When implementing LLM-based cognitive planning, consider these performance factors:

1. **Latency**: LLM inference can be slow; consider caching, local models, or hybrid approaches
2. **Reliability**: Plan for cases where LLMs return unexpected results
3. **Resource Usage**: LLMs can be computationally expensive
4. **Network Dependence**: Consider offline capabilities for robotic applications
5. **Safety**: Implement validation layers to ensure plans are safe to execute

## Summary

In this chapter, we've explored cognitive planning with LLMs for humanoid robotics applications. We've covered:

- The fundamentals of using LLMs for natural language understanding in robotics
- Techniques for task decomposition and hierarchical planning
- Methods for mapping natural language to ROS 2 actions
- Strategies for handling ambiguous or complex commands
- Practical implementation examples for LLM-based planning

These techniques enable humanoid robots to understand and execute complex natural language commands by breaking them down into sequences of executable actions. This builds on the voice recognition capabilities introduced in the [Voice-to-Action chapter](./chapter-1-voice-to-action.md) to create intelligent systems that can process natural language and generate appropriate robotic behaviors.

## Next Steps

In the next chapter, we'll explore the complete Vision-Language-Action pipeline by integrating the voice recognition from [Chapter 1](./chapter-1-voice-to-action.md) with the cognitive planning from this chapter, creating a full system where humanoid robots can receive voice commands, understand them using LLMs, and execute complex tasks in the physical world. The [Capstone: End-to-End VLA Pipeline](./chapter-3-vla-capstone.md) chapter will demonstrate how to combine all these components into a fully functional autonomous humanoid system.