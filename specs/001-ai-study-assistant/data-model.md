# Data Model: AI Study Assistant

## Message Entity
```
Message {
  id: string (unique identifier)
  content: string (message text content)
  role: "user" | "assistant" (sender type)
  timestamp: Date (when message was created)
}
```

## Conversation Entity
```
Conversation {
  id: string (unique identifier)
  messages: Message[] (ordered list of messages)
  createdAt: Date (when conversation started)
  lastActive: Date (when last message was sent)
  context: {
    pageTitle: string (title of page where conversation started)
    pageUrl: string (URL of page where conversation started)
    section: string (optional section information)
  }
}
```

## ChatRequest Entity
```
ChatRequest {
  message: string (user input message)
  context: {
    pageTitle: string (current page title)
    pageUrl: string (current page URL)
    section?: string (optional section information)
  }
}
```

## ChatResponse Entity
```
ChatResponse {
  response: string (AI-generated response)
  status: "success" | "error" (request status)
  contextUsed?: boolean (whether page context was used in response)
}
```

## Validation Rules
- Message content must not be empty
- Message role must be either "user" or "assistant"
- Timestamp must be within reasonable range
- Conversation must have at least one initial assistant message