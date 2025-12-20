# API Contract: Vision-Language-Action (VLA) Interface

## Overview
This document defines the API contracts for the Vision-Language-Action (VLA) integration system. These interfaces enable communication between the voice recognition, planning, and execution components of the VLA pipeline.

## Service: VoiceProcessing
**Purpose**: Convert speech input to text for command processing

### Operation: ProcessVoiceCommand
**Description**: Processes raw audio data and converts it to text using speech recognition

#### Request
```
{
  "audio_input": "bytes",
  "language_code": "string (optional, default: 'en')",
  "sample_rate": "integer (optional, default: 16000)",
  "encoding": "string (optional, default: 'wav')"
}
```

#### Response
```
{
  "success": "boolean",
  "transcribed_text": "string",
  "confidence": "float (0.0-1.0)",
  "language_detected": "string",
  "processing_time_ms": "integer",
  "error_message": "string (if success is false)"
}
```

#### Error Codes
- `400`: Invalid audio format or parameters
- `408`: Processing timeout
- `500`: Internal processing error
- `503`: Service unavailable

#### Performance Requirements
- Response time: < 200ms for 5-second audio clip
- Accuracy: > 90% for clear speech in quiet environment

## Service: TaskPlanning
**Purpose**: Convert natural language commands into executable task plans

### Operation: PlanTaskFromCommand
**Description**: Takes a natural language command and generates a sequence of actions to execute

#### Request
```
{
  "command_text": "string (required)",
  "robot_capabilities": "string (optional)",
  "environment_context": "string (optional)",
  "max_plan_steps": "integer (optional, default: 20)"
}
```

#### Response
```
{
  "success": "boolean",
  "plan_id": "string",
  "action_sequence": [
    {
      "action_type": "string (navigation|manipulation|perception|other)",
      "parameters": "object",
      "timeout": "float (seconds)",
      "fallback_actions": "array of action objects",
      "dependencies": "array of string (action IDs)"
    }
  ],
  "plan_confidence": "float (0.0-1.0)",
  "estimated_execution_time": "float (seconds)",
  "error_message": "string (if success is false)"
}
```

#### Error Codes
- `400`: Invalid command format or parameters
- `409`: Command conflicts with robot capabilities
- `422`: Unprocessable command (e.g., impossible task)
- `500`: Planning service error
- `504`: Planning timeout

#### Performance Requirements
- Response time: < 500ms for simple commands
- Response time: < 2000ms for complex commands
- Success rate: > 85% for common household commands

## Service: VLAPipeline
**Purpose**: End-to-end processing from voice command to robot action

### Operation: ExecuteVLACommand
**Description**: Complete pipeline execution from voice input to action completion

#### Request
```
{
  "audio_input": "bytes (optional - can be text instead)",
  "command_text": "string (optional - can be audio instead)",
  "timeout": "float (optional, default: 30.0 seconds)",
  "robot_id": "string (optional, default: primary robot)"
}
```

#### Response
```
{
  "success": "boolean",
  "execution_id": "string",
  "initial_plan": {
    "plan_id": "string",
    "action_sequence": "[array of action objects]",
    "status": "string"
  },
  "execution_status": "string (pending|executing|completed|failed|cancelled)",
  "progress": {
    "current_action": "string",
    "completed_actions": "integer",
    "total_actions": "integer",
    "estimated_remaining_time": "float (seconds)"
  },
  "error_message": "string (if success is false)"
}
```

#### Error Codes
- `400`: Invalid input format
- `408`: Execution timeout
- `409`: Robot busy or unavailable
- `422`: Command unprocessable
- `500`: Pipeline execution error

#### Performance Requirements
- End-to-end response time: < 1000ms for simple commands
- Success rate: > 80% for simple navigation and manipulation tasks

## Service: PerceptionInterface
**Purpose**: Interface with perception systems for environment awareness

### Operation: GetEnvironmentPerception
**Description**: Retrieve current perception data from robot sensors

#### Request
```
{
  "sensor_types": "array of string (optional, default: all)",
  "max_age_ms": "integer (optional, default: 1000)",
  "roi": {
    "center_x": "float",
    "center_y": "float",
    "center_z": "float",
    "radius": "float (meters)"
  }
}
```

#### Response
```
{
  "success": "boolean",
  "perception_data": {
    "timestamp": "datetime",
    "objects": [
      {
        "id": "string",
        "type": "string",
        "position": {"x": "float", "y": "float", "z": "float"},
        "confidence": "float (0.0-1.0)",
        "properties": "object"
      }
    ],
    "environment_map": "object (optional)",
    "quality_score": "float (0.0-1.0)"
  },
  "error_message": "string (if success is false)"
}
```

#### Performance Requirements
- Response time: < 100ms for cached data
- Response time: < 500ms for fresh sensor data
- Update frequency: At least 10Hz for dynamic environments

## Service: ActionExecution
**Purpose**: Execute specific actions on the robot

### Operation: ExecuteAction
**Description**: Execute a single action on the robot

#### Request
```
{
  "action_definition": {
    "action_type": "string",
    "parameters": "object",
    "timeout": "float (seconds)"
  },
  "robot_id": "string",
  "execution_id": "string (optional)"
}
```

#### Response
```
{
  "success": "boolean",
  "action_id": "string",
  "execution_status": "string (pending|executing|completed|failed)",
  "result": "object (action-specific)",
  "execution_time_ms": "integer",
  "error_message": "string (if success is false)"
}
```

#### Performance Requirements
- Command acceptance time: < 50ms
- Status update frequency: At least 1Hz during execution
- Timeout handling: Accurate within 100ms

## Data Validation Rules

### Text Validation
- Command text: 1-500 characters
- Language codes: Follow ISO 639-1 standard
- Confidence values: 0.0-1.0 range

### Numeric Validation
- Timeouts: Positive values only
- Coordinates: Valid float values
- IDs: Non-empty strings

### Object Validation
- Action parameters: Must match action type requirements
- Sensor types: Must be from predefined list
- Robot capabilities: Must be valid ROS 2 capabilities

## Security Considerations

### Authentication
- All services require authentication for production use
- API keys should be rotated regularly
- Rate limiting should be implemented per client

### Authorization
- Only authorized clients can execute actions on robots
- Command validation to prevent harmful actions
- Audit logging for all executed commands

## Error Handling

### Client Errors (4xx)
- Invalid request format
- Missing required parameters
- Unauthorized access
- Rate limiting exceeded

### Server Errors (5xx)
- Service unavailable
- Internal processing errors
- Timeout errors
- Dependency failures

### Recovery Strategies
- Retry with exponential backoff for temporary failures
- Fallback to simpler actions when complex ones fail
- Graceful degradation when services are unavailable