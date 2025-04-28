from flask import Flask, request, jsonify, render_template_string
import ollama

app = Flask(__name__)

# HTML content served at /
html_page = """
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>Ollama Chat</title>
  <style>
    body { font-family: Arial; background: #f2f2f2; padding: 20px; }
    #chat-box { border: 1px solid #ccc; background: #fff; padding: 10px; height: 300px; overflow-y: scroll; margin-bottom: 10px; }
    .message { margin: 5px 0; }
    .user { color: blue; }
    .bot { color: green; }
    #prompt { width: 80%; padding: 5px; }
    #send-btn { padding: 6px 12px; }
  </style>
</head>
<body>
  <h2>Ollama Chat</h2>
  <div id="chat-box"></div>
  <input type="text" id="prompt" placeholder="Ask a question..." />
  <button id="send-btn">Send</button>

  <script>
    const chatBox = document.getElementById('chat-box');
    const promptInput = document.getElementById('prompt');
    const sendBtn = document.getElementById('send-btn');

    function appendMessage(sender, text) {
      const messageDiv = document.createElement('div');
      messageDiv.className = 'message ' + sender;
      messageDiv.textContent = `${sender === 'user' ? 'You' : 'Ollama'}: ${text}`;
      chatBox.appendChild(messageDiv);
      chatBox.scrollTop = chatBox.scrollHeight;
    }

    sendBtn.addEventListener('click', async () => {
      const prompt = promptInput.value.trim();
      if (!prompt) return;

      appendMessage('user', prompt);
      promptInput.value = '';

      try {
        const response = await fetch('/process', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ prompt })
        });

        const data = await response.json();
        appendMessage('bot', data.response);
      } catch (error) {
        appendMessage('bot', 'Error contacting the server.');
        console.error(error);
      }
    });

    promptInput.addEventListener('keypress', (e) => {
      if (e.key === 'Enter') sendBtn.click();
    });
  </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(html_page)

@app.route('/process', methods=['POST'])
def process():
    data = request.json
    prompt = data['prompt']
    mode = data.get('mode', 'chat')

    try:
        response = ollama.chat(
            model='deepseek-r1:1.5b',  # or any model you've pulled
            messages=[{
                "role": "user",
                "content": f"{mode} mode: {prompt}"
            }]
        )
        return jsonify({"response": response['message']['content']})
    except Exception as e:
        return jsonify({"response": f"Error: {str(e)}"}), 500

if __name__ == '__main__':
    app.run(port=1198)
