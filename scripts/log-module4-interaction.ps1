# PowerShell script to log Module 4 interactions to timestamped markdown files
# Usage: .\log-module4-interaction.ps1 "user_prompt" "system_decision" "model_response"

param(
    [string]$UserPrompt = "",
    [string]$SystemDecision = "",
    [string]$ActionPlan = "",
    [string]$ModelResponse = ""
)

# Get the current timestamp
$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"

# Create the history/module-4 directory if it doesn't exist
$module4Dir = Join-Path -Path "history" -ChildPath "module-4"
if (!(Test-Path $module4Dir)) {
    New-Item -ItemType Directory -Path $module4Dir -Force
}

# Create the markdown file with timestamp
$logFileName = "${timestamp}_module4_interaction.md"
$logFilePath = Join-Path -Path $module4Dir -ChildPath $logFileName

# Create the content for the markdown file
$content = @"
# Module 4 Interaction Log - $timestamp

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

Write-Output "Interaction logged to: $logFilePath"