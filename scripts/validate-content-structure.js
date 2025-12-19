const fs = require('fs');
const path = require('path');
const matter = require('gray-matter');

const MODULES_DIR = path.join(__dirname, '../apps/web/modules');

const REQUIRED_SECTIONS = [
  'Learning Objectives',
  'Prerequisites',
  'Core Concepts',
  // 'Examples / Exercises', // Can be Examples OR Exercises
  'Summary',
  'References'
];

function getHeadings(content) {
  const headings = [];
  const lines = content.split(/\r?\n/);
  for (const line of lines) {
    const match = line.match(/^##\s+(.+)$/);
    if (match) {
      headings.push(match[1].trim());
    }
  }
  return headings;
}

function checkExamplesOrExercises(headings) {
  return headings.some(h => 
    h === 'Examples' || h === 'Exercises' || h === 'Examples / Exercises'
  );
}

function validateFile(filePath) {
  const content = fs.readFileSync(filePath, 'utf8');
  const { content: markdown } = matter(content);
  const headings = getHeadings(markdown);
  console.log('Found headings in ' + filePath, headings);

  const missing = [];
  for (const section of REQUIRED_SECTIONS) {
    if (!headings.includes(section)) {
      missing.push(section);
    }
  }

  if (!checkExamplesOrExercises(headings)) {
    missing.push('Examples OR Exercises');
  }

  if (missing.length > 0) {
    return { valid: false, missing };
  }
  return { valid: true };
}

function walkDir(dir, callback) {
  fs.readdirSync(dir).forEach(f => {
    let dirPath = path.join(dir, f);
    let isDirectory = fs.statSync(dirPath).isDirectory();
    isDirectory ? walkDir(dirPath, callback) : callback(path.join(dir, f));
  });
}

console.log('Validating content structure...');
let hasError = false;

walkDir(MODULES_DIR, (filePath) => {
  if (!filePath.endsWith('.md') && !filePath.endsWith('.mdx')) return;
  // Skip Intro/Index pages if they are not chapters
  // Heuristic: If filename is index.md, it might be a module intro which has different rules
  if (path.basename(filePath) === 'index.md') return;

  const result = validateFile(filePath);
  if (!result.valid) {
    console.error(`❌ ${path.relative(MODULES_DIR, filePath)} missing: ${result.missing.join(', ')}`);
    hasError = true;
  } else {
    // console.log(`✅ ${path.relative(MODULES_DIR, filePath)}`);
  }
});

if (hasError) {
  console.log('Validation FAILED.');
  process.exit(1);
} else {
  console.log('All chapters valid.');
}
