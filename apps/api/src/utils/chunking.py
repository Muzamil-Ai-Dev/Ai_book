import hashlib
import re
from typing import List
from src.models.content import ContentChunk, SourceMetadata

def generate_chunk_id(path: str, heading: str, content: str) -> str:
    """Generates a deterministic ID for a chunk."""
    hash_input = f"{path}:{heading}:{content[:100]}"
    return hashlib.md5(hash_input.encode()).hexdigest()

def chunk_markdown(content: str, path: str, module: str, chapter: str) -> List[ContentChunk]:
    """
    Chunks a markdown file based on heading structure.
    Simple implementation splitting by H1, H2, H3.
    """
    chunks = []
    lines = content.split('\n')
    current_heading = "Introduction"
    current_text = []
    order = 0
    
    for line in lines:
        if line.startswith(('# ', '## ', '### ')):
            # Save previous chunk if exists
            if current_text:
                text_content = '\n'.join(current_text).strip()
                if text_content:
                    chunk_id = generate_chunk_id(path, current_heading, text_content)
                    chunks.append(ContentChunk(
                        id=chunk_id,
                        text=text_content,
                        metadata=SourceMetadata(
                            module=module,
                            chapter=chapter,
                            heading=current_heading,
                            path=path
                        ),
                        order=order
                    ))
                    order += 1
            
            # Start new chunk
            current_heading = line.lstrip('#').strip()
            current_text = []
        else:
            current_text.append(line)
            
    # Add final chunk
    if current_text:
        text_content = '\n'.join(current_text).strip()
        if text_content:
            chunk_id = generate_chunk_id(path, current_heading, text_content)
            chunks.append(ContentChunk(
                id=chunk_id,
                text=text_content,
                metadata=SourceMetadata(
                    module=module,
                    chapter=chapter,
                    heading=current_heading,
                    path=path
                ),
                order=order
            ))
            
    return chunks
