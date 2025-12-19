const fs = require('fs');
const path = require('path');

const CONTENT_DIR = path.join(__dirname, '../content/modules');

function walkDir(dir, callback) {
  fs.readdirSync(dir).forEach(f => {
    let dirPath = path.join(dir, f);
    let isDirectory = fs.statSync(dirPath).isDirectory();
    isDirectory ? walkDir(dirPath, callback) : callback(path.join(dir, f));
  });
}

function forceQuotes(filePath) {
  if (!filePath.endsWith('.md') && !filePath.endsWith('.mdx')) return;

  let content = fs.readFileSync(filePath, 'utf8');
  let lines = content.split('\n');
  let changed = false;
  let fmCount = 0;

  for (let i = 0; i < lines.length; i++) {
    let line = lines[i];
    if (line.trim() === '---') {
      fmCount++;
      if (fmCount >= 2) break; // End of frontmatter
      continue;
    }

    if (fmCount === 1) { // Inside frontmatter
        const match = line.match(/^(\w+):\s*(.+)$/);
        if (match) {
            const key = match[1];
            let value = match[2].trim();

            if (key === 'title' || key === 'description' || key === 'sidebar_label') {
                // If not already quoted
                if (!value.startsWith('"') && !value.startsWith("'")) {
                    // If contains dangerous chars
                    if (value.includes(':') || value.includes('#') || value.includes('(') || value.includes(')')) {
                        // Escape internal double quotes
                        const escapedValue = value.replace(/"/g, '\\"');
                        lines[i] = `${key}: "${escapedValue}"`;
                        changed = true;
                    }
                }
            }
        }
    }
  }

  if (changed) {
    console.log(`Fixing frontmatter in: ${filePath}`);
    fs.writeFileSync(filePath, lines.join('\n'));
  }
}

console.log('Running Quote Enforcer...');
walkDir(CONTENT_DIR, forceQuotes);
console.log('Done.');
