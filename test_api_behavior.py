import os
import time
from openai import OpenAI

api_key = os.environ.get("OPENROUTER_API_KEY")
client = OpenAI(api_key=api_key, base_url="https://openrouter.ai/api/v1")

# The final 6 models to be tested
MODELS_TO_TEST = [
    "openai/gpt-5.5",
    "deepseek/deepseek-v4-pro",
    "google/gemini-3.1-pro-preview",
    "anthropic/claude-opus-4.6",
    "qwen/qwen3.5-plus-02-15",
    "z-ai/glm-5-turbo"
]

# The three reasoning settings to test
REASONING_SETTINGS = ["default", "none", "low"]

def run_test():
    if not api_key:
        print("CRITICAL: Please export OPENROUTER_API_KEY first.")
        return

    print("Starting reasoning_effort compatibility test for all models...\n")

    for model in MODELS_TO_TEST:
        print(f"{'='*70}")
        print(f" 🚀 Testing Model: {model} ")
        print(f"{'='*70}")
        
        for setting in REASONING_SETTINGS:
            print(f"\n▶ [ Testing Parameter: reasoning_effort = '{setting}' ]")
            
            kwargs = {
                "model": model,
                "messages": [{"role": "user", "content": "Write a Python Hello World program. Return ONLY the code, no explanations."}],
                "temperature": 0.0
            }
            
            # If not default, add the parameter to extra_body
            if setting != "default":
                kwargs["extra_body"] = {"reasoning_effort": setting}
            
            start_time = time.time()
            try:
                response = client.chat.completions.create(**kwargs)
                end_time = time.time()
                
                content = response.choices[0].message.content
                tokens = response.usage.total_tokens
                
                print(f"  ✅ Success! Time elapsed: {end_time - start_time:.2f}s | Total Tokens: {tokens}")
                
                # Check if there are hidden thinking processes in the output
                if "<think>" in content or "thinking" in content.lower():
                    print("  ⚠️ WARNING: Found <think> tag in output, meaning the model is still reasoning in the background!")
                
                # Print the first 60 characters to see if it outputs directly or includes verbose text
                preview = content.replace('\n', ' ')[:60]
                print(f"  📝 Output Preview: {preview}...")
                
            except Exception as e:
                # Catch exceptions to see if the model crashes due to unrecognized none/low values
                print(f"  ❌ Error Caught: {str(e)[:150]}...")

if __name__ == "__main__":
    run_test()
