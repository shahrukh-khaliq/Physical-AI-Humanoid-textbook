#!/bin/bash

# Script to log Module 4 interactions to timestamped markdown files
# Usage: ./log-module4-interaction.sh "user_prompt" "system_decision" "model_response"

# Get the current timestamp
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")

# Create the history/module-4 directory if it doesn't exist
mkdir -p history/module-4

# Create the markdown file with timestamp
LOG_FILE="history/module-4/${TIMESTAMP}_module4_interaction.md"

# Write the content to the markdown file
cat <<EOF > "$LOG_FILE"
# Module 4 Interaction Log - $TIMESTAMP

## Prompt
$user_prompt

## Reasoning
$system_decision

## Action Plan
$action_plan

## Output
$model_response

## Metadata
- Timestamp: $TIMESTAMP
- File: $LOG_FILE
EOF

echo "Interaction logged to: $LOG_FILE"