# PowerShell script to log Vision-Language-Action (VLA) interactions to timestamped markdown files
# Usage: .\log-vla-interaction.ps1 "user_prompt" "system_decision" "model_response"

param(
    [string]$UserPrompt = "",
    [string]$SystemDecision = "",
    [string]$ActionPlan = "",
    [string]$ModelResponse = ""
)

# Get the current timestamp
$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"

# Create the history/Vision-Language-Action (VLA) directory if it doesn't exist
$vlaDir = Join-Path -Path "history" -ChildPath "Vision-Language-Action (VLA)"
if (!(Test-Path $vlaDir)) {
    New-Item -ItemType Directory -Path $vlaDir -Force
}

# Create the markdown file with timestamp
$logFileName = "${timestamp}_vla_interaction.md"
$logFilePath = Join-Path -Path $vlaDir -ChildPath $logFileName

# Create the content for the markdown file
$content = @"
# Vision-Language-Action (VLA) Interaction Log - $timestamp

## Prompt
$UserPrompt

## Reasoning
$SystemDecision

## Action Plan
$ActionPlan

## Output
$ModelResponse

## Metadata
- Timestamp: $timestamp
- File: $logFilePath
"@

# Write the content to the markdown file
Set-Content -Path $logFilePath -Value $content

Write-Output "VLA interaction logged to: $logFilePath"