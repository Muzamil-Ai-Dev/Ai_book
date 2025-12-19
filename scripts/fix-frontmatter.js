const fs = require('fs');
const path = require('path');
const matter = require('gray-matter');

const CONTENT_DIR = path.join(__dirname, '../content/modules');

function walkDir(dir, callback) {
  fs.readdirSync(dir).forEach(f => {
    let dirPath = path.join(dir, f);
    let isDirectory = fs.statSync(dirPath).isDirectory();
    isDirectory ? walkDir(dirPath, callback) : callback(path.join(dir, f));
  });
}

function fixFrontmatter(filePath) {
  if (!filePath.endsWith('.md') && !filePath.endsWith('.mdx')) return;

  const content = fs.readFileSync(filePath, 'utf8');
  const parsed = matter(content);
  let changed = false;

  // Check title
  if (parsed.data.title && typeof parsed.data.data.title === 'string') {
    if (parsed.data.title.includes(':') || parsed.data.title.includes('(') || parsed.data.title.includes(')')) {
      // We can't just change the object and dump it, because gray-matter might reformat weirdly.
      // But let's try just forcing quotes by wrapping in simple quote logic if we were modifying raw text.
      // Actually, dumping with forceQuotes might be easier.
    }
  }

  // Regex approach is safer to preserve other formatting
  // Look for title: value where value has : and isn't quoted
  
  let lines = content.split('\n');
  let inFrontmatter = false;
  let fmCount = 0;
  let newLines = lines.map(line => {
    if (line.trim() === '---') {
      fmCount++;
      inFrontmatter = fmCount < 2;
      return line;
    }
    
    if (inFrontmatter) {
      const match = line.match(/^(\w+):\s*(.+)$/);
      if (match) {
        const key = match[1];
        let value = match[2].trim();
        
        if ((key === 'title' || key === 'description') && !value.startsWith('"') && !value.startsWith("'")) {
             // Check for problematic chars
             if (value.includes(':') || value.includes('#') || value.includes('(') || value.includes(')')) {
                 // Escape existing quotes
                 value = value.replace(/"/g, '\\"');
                 return `${key}: "${value}"`;
             }
        }
      }
    }
    return line;
  });

  const newContent = newLines.join('\n');
  if (newContent !== content) {
      console.log(`Fixing ${filePath}`);
      fs.writeFileSync(filePath, newContent);
  }
}

walkDir(CONTENT_DIR, fixFrontmatter);
console.log('Frontmatter fix complete.');
