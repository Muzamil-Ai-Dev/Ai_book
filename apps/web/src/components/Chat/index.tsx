import React, { useState, useEffect, useRef } from 'react';
import ReactMarkdown from 'react-markdown';
import styles from './styles.module.css';

interface Message {
  role: 'user' | 'assistant';
  content: string;
}

const Chat: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [input, setInput] = useState('');
  const [messages, setMessages] = useState<Message[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

      const handleSubmit = async (e: React.FormEvent) => {

      e.preventDefault();

      if (!input.trim() || isLoading) return;

  

      const userMessage: Message = { role: 'user', content: input };

      setMessages(prev => [...prev, userMessage]);

      setInput('');

      setIsLoading(true);

  

      // Default to local, but easy to swap for production URL

      const API_URL = process.env.NODE_ENV === 'development' 
        ? 'http://127.0.0.1:8000' 
        : 'https://muzamil-ai-dev-ai-book.hf.space'; // Replace with your actual HF Space Direct URL if different

  

      try {

        const response = await fetch(`${API_URL}/query`, {

          method: 'POST',

          headers: { 'Content-Type': 'application/json' },

          body: JSON.stringify({ question: input }),

        });

  
      const data = await response.json();
      const assistantMessage: Message = { role: 'assistant', content: data.answer };
      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Error fetching chat:', error);
      setMessages(prev => [...prev, { role: 'assistant', content: 'Sorry, I am having trouble connecting to the brain. Make sure the backend is running!' }]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.chatWrapper}>
      {isOpen ? (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <h3>AI Tutor</h3>
            <button onClick={() => setIsOpen(false)} className={styles.closeBtn}>×</button>
          </div>
          <div className={styles.messageList}>
            {messages.length === 0 && (
              <div className={styles.welcome}>
                👋 Hello! I am your Physical AI tutor. Ask me anything about this book!
              </div>
            )}
            {messages.map((msg, i) => (
              <div key={i} className={`${styles.message} ${msg.role === 'user' ? styles.userMessage : styles.botMessage}`}>
                <ReactMarkdown>{msg.content}</ReactMarkdown>
              </div>
            ))}
            {isLoading && <div className={styles.loading}>Generating...</div>}
            <div ref={messagesEndRef} />
          </div>
          <form onSubmit={handleSubmit} className={styles.chatInput}>
            <input
              type="text"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              placeholder="Ask a question..."
              autoFocus
            />
            <button type="submit" disabled={isLoading}>Send</button>
          </form>
        </div>
      ) : (
        <button onClick={() => setIsOpen(true)} className={styles.chatLauncher}>
          💬 Chat with Book
        </button>
      )}
    </div>
  );
};

export default Chat;
