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

function check(filePath) {
  if (!filePath.endsWith('.md') && !filePath.endsWith('.mdx')) return;

  const content = fs.readFileSync(filePath, 'utf8');
  try {
    const parsed = matter(content);
    // Explicitly check if title/description act like objects (which happens if unquoted colon)
    if (typeof parsed.data.title === 'object') {
        throw new Error(`Title parsed as object: ${JSON.stringify(parsed.data.title)}`);
    }
    if (typeof parsed.data.description === 'object') {
        throw new Error(`Description parsed as object: ${JSON.stringify(parsed.data.description)}`);
    }
  } catch (err) {
    console.error(`❌ FAILED: ${filePath}`);
    console.error(err.message);
  }
}

console.log('Validating Frontmatter...');
walkDir(CONTENT_DIR, check);
console.log('Done.');
